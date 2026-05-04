#include "aperture_tuning.h"
#include <string.h>
#include <stdlib.h>
#include "platform_i2c.h"
#include "xil_printf.h"

/* --- Platform-specific I2C hooks (replace these) --- */
/* These are simple prototypes; implement for your MCU */
extern int platform_i2c_set_mux(uint8_t bus); /* select I2C MUX channel */
extern int platform_i2c_write(uint8_t addr, uint8_t reg, const uint8_t *data, uint8_t len);
/* If using async DMA/interrupt I2C, implement platform_i2c_write_async... */

/* --- internal mapping: mirrors your Python configs --- */
/* Each IO expander: {bus, address, channelMask} channelMask: bitwise (1 = chanA present, 2 = chanB present) */
typedef struct {
    uint8_t bus;
    uint8_t addr;
    uint8_t chan_mask;
} ioexp_map_t;

/* Default mapping - adjust to match IOEXP_SETUP_CONFIG */
static const ioexp_map_t ioexp_map[NUM_IO_EXPANDERS] = {
    {IO_EXP_BUS0, IO_EXP_ADDR1, 3}, /* 0 => A+B */
    {IO_EXP_BUS0, IO_EXP_ADDR2, 3}, /* 1 */
    {IO_EXP_BUS1, IO_EXP_ADDR1, 3}, /* 2 */
    {IO_EXP_BUS1, IO_EXP_ADDR2, 3}, /* 3 */
    {IO_EXP_BUS2, IO_EXP_ADDR1, 3}, /* 4 */
    {IO_EXP_BUS2, IO_EXP_ADDR2, 3}, /* 5 */
    {IOEXP_TX_BUS, IO_EXP_ADDR1, 1} /* 6: TX only on chanA */
};

/* Precomputed payloads: [preset_index][expander_index] => 3 bytes */
static ioexp_payload_t *precomputed_payloads = NULL;
static uint8_t precomputed_num_presets = 0;

/* helper: allocate/clear precomputed storage */
static int alloc_precomputed_storage(uint8_t num_presets)
{
    if (precomputed_payloads) free(precomputed_payloads);
    precomputed_payloads = malloc((size_t)num_presets * NUM_IO_EXPANDERS * sizeof(ioexp_payload_t));
    if (!precomputed_payloads) return -1;
    memset(precomputed_payloads, 0, (size_t)num_presets * NUM_IO_EXPANDERS * sizeof(ioexp_payload_t));
    precomputed_num_presets = num_presets;
    return 0;
}

/* reverse bits of value `x` for `bits` length (LSB <-> MSB) */
static uint32_t reverse_bits_uint32(uint32_t x, uint8_t bits)
{
    uint32_t r = 0;
    for (uint8_t i = 0; i < bits; ++i) {
        r <<= 1;
        r |= (x & 1);
        x >>= 1;
    }
    return r;
}

/* Public utility: encode a channel to a 12-bit integer */
uint16_t atc_encode_channel_config(const aperture_channel_config_t *cfg)
{
    uint32_t tuning = cfg->tuning & ((1u << TUNING_MAX_BITS) - 1u);
    uint32_t matching = cfg->matching & ((1u << MATCHING_MAX_BITS) - 1u);
    uint32_t det = cfg->detune_enable & 0x1u;

    uint32_t reversed_tuning = reverse_bits_uint32(tuning, TUNING_MAX_BITS); /* 7 bits reversed */
    uint16_t encoded = (uint16_t)((matching << MATCHING_OFFSET) |
                                 (reversed_tuning << TUNING_OFFSET) |
                                 (det << DETUNE_ENABLE_OFFSET));
    return encoded & 0x0FFFu;
}

/* Public: decode encoded 12-bit value into channels */
void atc_decode_channel_config(uint16_t encoded, aperture_channel_config_t *out)
{
    uint16_t matching_val = (encoded >> MATCHING_OFFSET) & 0x0F;
    uint16_t reversed_tuning_val = (encoded >> TUNING_OFFSET) & 0x7F;
    uint16_t det_val = (encoded >> DETUNE_ENABLE_OFFSET) & 0x1;

    uint32_t tuning_val = (uint32_t)reverse_bits_uint32(reversed_tuning_val, TUNING_MAX_BITS);
    out->tuning = (uint8_t)(tuning_val & 0x7F);
    out->matching = (uint8_t)matching_val;
    out->detune_enable = (uint8_t)det_val;
}

/* internal helper to compute 3-byte payload from two 12-bit channel encodings */
/* order: combined = (chanB << 12) | chanA */
static void assemble_payload_from_chan12(uint16_t chanA_enc, uint16_t chanB_enc, ioexp_payload_t *out)
{
    uint32_t combined = ((uint32_t)chanB_enc << 12) | ((uint32_t)chanA_enc & 0x0FFFu);
    uint8_t b1 = combined & 0xFFu;
    uint8_t b2 = (combined >> 8) & 0xFFu;
    uint8_t b3 = (combined >> 16) & 0xFFu;
    out->bytes[0] = b1;
    out->bytes[1] = b2;
    out->bytes[2] = b3;
}

/* New signature: presets has NUM_CHANNELS_RX + 1 entries per preset:
   index 0 = TX, indices 1..NUM_CHANNELS_RX = RX1..RXN */
int atc_precompute_sets(const aperture_channel_config_t presets[][NUM_CHANNELS],
                        uint8_t num_presets)
{
    if (num_presets == 0 || num_presets > MAX_PRESETS) return -1;
    if (alloc_precomputed_storage(num_presets) != 0) return -2;

    /* For each preset, compute payloads for each IO expander */
    for (uint8_t p = 0; p < num_presets; ++p) {
        for (uint8_t ex = 0; ex < NUM_IO_EXPANDERS; ++ex) {
            const ioexp_map_t *map = &ioexp_map[ex];
            ioexp_payload_t payload = { .bytes = {0,0,0} };

            /* Hardcoded mapping table to match your Python TUNING_SETUP_CONFIG (index 0==ch1).
               Format: {bus, addr, chanb, active}
               If your hardware mapping differs, replace this table with the real mapping.
            */
            static const struct { uint8_t bus; uint8_t addr; uint8_t chanb; uint8_t active; } rx_channel_map[NUM_CHANNELS_RX] = {
                {IO_EXP_BUS0, IO_EXP_ADDR1, 0, ACTIVE_CHANNEL}, /* ch1 */
                {IO_EXP_BUS0, IO_EXP_ADDR1, 1, ACTIVE_CHANNEL}, /* ch2 */
                {IO_EXP_BUS1, IO_EXP_ADDR1, 0, ACTIVE_CHANNEL}, /* ch3 */
                {IO_EXP_BUS1, IO_EXP_ADDR1, 1, ACTIVE_CHANNEL}, /* ch4 */
                {IO_EXP_BUS1, IO_EXP_ADDR2, 0, ACTIVE_CHANNEL}, /* ch5 */
                {IO_EXP_BUS1, IO_EXP_ADDR2, 1, ACTIVE_CHANNEL}, /* ch6 */
                {IO_EXP_BUS0, IO_EXP_ADDR2, 0, INACTIVE_CHANNEL},/* ch7 */
                {IO_EXP_BUS0, IO_EXP_ADDR2, 1, INACTIVE_CHANNEL},/* ch8 */
                {IO_EXP_BUS2, IO_EXP_ADDR1, 0, INACTIVE_CHANNEL},/* ch9 */
                {IO_EXP_BUS2, IO_EXP_ADDR1, 1, INACTIVE_CHANNEL},/* ch10 */
                {IO_EXP_BUS2, IO_EXP_ADDR2, 0, INACTIVE_CHANNEL},/* ch11 */
                {IO_EXP_BUS2, IO_EXP_ADDR2, 1, INACTIVE_CHANNEL} /* ch12 */
            };

            /* Find chanA_index & chanB_index matching this expander mapping.
               chanA_index/chanB_index are indices 0..(NUM_CHANNELS_RX-1) for RX channels.
            */
            int chanA_index = -1;
            int chanB_index = -1;
            for (int i = 0; i < NUM_CHANNELS_RX; ++i) {
                if (rx_channel_map[i].bus == map->bus && rx_channel_map[i].addr == map->addr) {
                    if (rx_channel_map[i].chanb == 0) chanA_index = i; else chanB_index = i;
                }
            }

            uint16_t chanA_enc = 0;
            uint16_t chanB_enc = 0;

            /* If this expander is the TX expander, use presets[p][0] as TX config (first element) */
            if (map->bus == IOEXP_TX_BUS) {
                const aperture_channel_config_t *tx_cfg = &presets[p][0];
                chanA_enc = atc_encode_channel_config(tx_cfg);
                chanB_enc = 0;
            } else {
                /* For RX expanders, pick the rx config from presets[p][1 + rx_index] */
                if (chanA_index >= 0) {
                    const aperture_channel_config_t *cfg = &presets[p][1 + chanA_index];
                    chanA_enc = atc_encode_channel_config(cfg);
                } else {
                    chanA_enc = 0;
                }
                if (chanB_index >= 0) {
                    const aperture_channel_config_t *cfg = &presets[p][1 + chanB_index];
                    chanB_enc = atc_encode_channel_config(cfg);
                } else {
                    chanB_enc = 0;
                }
            }

            assemble_payload_from_chan12(chanA_enc, chanB_enc, &payload);

            /* store into precomputed array */
            precomputed_payloads[p * NUM_IO_EXPANDERS + ex] = payload;
        }
    }

    return 0;
}

/* Return pointer to precomputed payload */
const ioexp_payload_t* atc_get_payload_ptr(uint8_t preset_index, uint8_t expander_index)
{
    if (!precomputed_payloads) return NULL;
    if (preset_index >= precomputed_num_presets) return NULL;
    if (expander_index >= NUM_IO_EXPANDERS) return NULL;
    return &precomputed_payloads[preset_index * NUM_IO_EXPANDERS + expander_index];
}

/* Number of expanders */
uint8_t atc_num_io_expanders(void) { return NUM_IO_EXPANDERS; }

/* Blocking application: loop expanders, set MUX, write 3 bytes to outputs register */
int atc_apply_preset_blocking(uint8_t preset_index)
{
    if (!precomputed_payloads) return -1;
    if (preset_index >= precomputed_num_presets) return -2;

    for (uint8_t ex = 0; ex < NUM_IO_EXPANDERS; ++ex) {
        const ioexp_map_t *map = &ioexp_map[ex];
        const ioexp_payload_t *pl = atc_get_payload_ptr(preset_index, ex);
        if (!pl) continue;
        /* Set MUX to appropriate bus */
        if (platform_i2c_write_mux(map->bus) != 0) {
            /* handle mux error */
            continue;
            // return -3;
        }
        /* write to IO_EXP_OUTPUTS_REG */
        if (platform_i2c_write(map->addr, IO_EXP_OUTPUTS_REG, pl->bytes, 3) != 0) {
            /* handle write error */
            continue;
            // return -4;
        }

        xil_printf("AT Control: %02x %02x %02x\r\n", (unsigned int)pl->bytes[0], (unsigned int)pl->bytes[1], (unsigned int)pl->bytes[2]);
    }
    return 0;
}

/* Placeholder async API - actual implementation depends on your platform */
int atc_apply_preset_async(uint8_t preset_index)
{
    /* Ideally push preset_index into a queue that an I2C transfer task processes.
       To keep this file generic, we just call blocking implementation.
       Replace this with a non-blocking dispatcher for production. */
    return atc_apply_preset_blocking(preset_index);
}

/* Initialization placeholder - sets up I2C, optionally config/invert regs on expanders */
const uint8_t outputs_register_config[3] = {0x00, 0x00, 0x00};
const uint8_t inversion_register_config[3] = {0xFF, 0xFF, 0xFF};

int atc_init(void)
{
    /* If you need to configure expanders (set config/invert registers), do it here once.
       For example: for each expander set platform_i2c_set_mux(map.bus) then write config registers. */

    for (uint8_t ex = 0; ex < NUM_IO_EXPANDERS; ++ex) {
        const ioexp_map_t *map = &ioexp_map[ex];

        /* Set MUX to appropriate bus */
        if (platform_i2c_write_mux(map->bus) != 0) {
            /* handle mux error */
            continue;
            // return -3;
        }

        if (platform_i2c_write(map->addr, IO_EXP_CONFIG_REG, outputs_register_config, 3) != 0) {
            /* handle write error */
            continue;
            // return -4;
        }

        if (platform_i2c_write(map->addr, IO_EXP_INVERT_REG, inversion_register_config, 3) != 0) {
            /* handle write error */
            continue;
            // return -4;
        }
    }
    return 0;
}

/* de-init freeing storage */
void atc_deinit(void)
{
    if (precomputed_payloads) {
        free(precomputed_payloads);
        precomputed_payloads = NULL;
    }
    precomputed_num_presets = 0;
}

