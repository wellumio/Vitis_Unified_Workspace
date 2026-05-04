#include "xparameters.h"
#include "xstatus.h"
#include "xintc.h"
#include "xil_printf.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xipipsu.h"
#include "xil_cache.h"
#include "xil_mpu.h"
#include "sleep.h"
#include <stdint.h>
#include <xgpio.h>
#include <xil_types.h>
#include "xiic.h"
#include "xiic_l.h"
#include "aperture_tuning.h"
#include "platform_i2c.h"
#include "xgpio.h"
#include "xgpio_l.h"
#include <math.h>

static XIntc   Intc;
static XIpiPsu IpiInst;
static XScuGic GicInst;

static XGpio   Gpio_DDS_Chan;
static XGpio   Gpio_Sample;
static XGpio   Gpio_Tui_Trigger;

#define XPAR_XIPIPSU_0_DEVICE_ID    0
#define IPI_DEV_ID                  XPAR_XIPIPSU_0_DEVICE_ID
#define IPI_INT_ID                  XPAR_XIPIPSU_0_INTR
#define IPI_MASK_SELF               XPAR_XIPIPSU_0_IPI_BITMASK
#define IPI_MASK_APU                XPAR_IPI1_0_IPI_BITMASK
#define GIC_DEVICE_ID               XPAR_SCUGIC_SINGLE_DEVICE_ID
#define FABRIC_INTC_IRQ_ID          136
#define SHARED_MEM_ADDR             0x7FFFF000U
#define IPI_MAGIC                   0xABCD1234U
#define IPI_MAX_PAYLOAD_BYTES       512
#define XPAR_AXI_GPIO_0_DEVICE_ID   0
#define GPIO_DEVICE_ID              XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_READ_CH                1   // channel 1 used for input
#define GPIO_WRITE_CH               2   // channel 2 used for output
#define GPIO_CH               1   // channel 2 used for output

#define TUI_TRIGGER_US              500
#define NUM_PRESETS 7
aperture_channel_config_t presets[NUM_PRESETS][NUM_CHANNELS];
#define XPAR_AXI_IIC_0_DEVICE_ID    0

typedef struct {
    uint32_t magic;    // sanity check, e.g. 0xABCD1234
    uint32_t cmd;      // application-specific command ID
    uint32_t len;      // number of valid bytes in payload (<= IPI_MAX_PAYLOAD_BYTES)
    uint32_t status;   // 0 = empty, 1 = request ready, 2 = response ready, etc.
    uint8_t  payload[IPI_MAX_PAYLOAD_BYTES];
} ipi_msg_t;

volatile bool ipi_message_flag = false;
ipi_msg_t ipi_buffer;

/* ISR-visible state */
static volatile unsigned irq_count = 0;

/* globals shared between ISR and main */
volatile uint8_t pending_preset = 0xFF;   /* 0xFF = none */
volatile uint8_t aperture_tuning_change_flag = 0;
volatile uint8_t pulse_tui_start_flag = 0;


static void DeviceDriverHandler(void *Ref)
{
    (void)Ref;
    irq_count++;
    uint8_t pl_value = (uint8_t)XGpio_DiscreteRead(&Gpio_DDS_Chan, GPIO_CH);
    pending_preset = pl_value;
    aperture_tuning_change_flag = 1;
    // Clear interrupt
    uint32_t status = XGpio_InterruptGetStatus(&Gpio_DDS_Chan);
    XGpio_InterruptClear(&Gpio_DDS_Chan, status);
    // xil_printf("Interrupt\r\n");

    __asm__ volatile ("dmb" ::: "memory");
}


static void Map_PlIo(void)
{
    Xil_DCacheDisable();
    Xil_ICacheDisable();
    Xil_DisableMPU();
    Xil_SetMPURegion(0xA0000000ULL, REGION_512M,
                     DEVICE_NONSHARED | PRIV_RW_USER_RW | EXECUTE_NEVER);
    Xil_EnableMPU();
    Xil_ICacheEnable();
    Xil_DCacheEnable();
}


void IpiHandler(void *CallBackRef)
{
    XIpiPsu *IpiPtr = (XIpiPsu *)CallBackRef;
    ipi_msg_t *msg = (ipi_msg_t *)SHARED_MEM_ADDR;
    XIpiPsu_ClearInterruptStatus(IpiPtr, IPI_MASK_SELF);
    Xil_DCacheInvalidateRange((UINTPTR)msg, sizeof(ipi_msg_t));
    if (msg->magic == IPI_MAGIC && msg->status == 1 && msg->cmd != 1) {
        memcpy((void *)&ipi_buffer, msg, sizeof(ipi_msg_t));
        ipi_message_flag = true;
        msg->status = 2;  // mark as "response ready"
    }
    else {

        int Status;
        uint8_t mux_val;
        Status = platform_i2c_read_mux(&mux_val);
        msg->payload[0] = Status;
        msg->payload[1] = 0x70;
        msg->payload[2] = mux_val;
        msg->len = 3;
        msg->status = 2;  // mark as "response ready"
    }
    Xil_DCacheFlushRange((UINTPTR)msg, sizeof(ipi_msg_t));
    XIpiPsu_TriggerIpi(IpiPtr, IPI_MASK_APU);
}


static int InitIpi(XIpiPsu *IpiPtr)
{
    XIpiPsu_Config *CfgPtr;
    int Status;

    CfgPtr = XIpiPsu_LookupConfig(IPI_DEV_ID);
    if (CfgPtr == NULL) {
        return XST_FAILURE;
    }

    Status = XIpiPsu_CfgInitialize(IpiPtr, CfgPtr, CfgPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    /* Enable interrupt to be generated when our IPI bit is set */
    XIpiPsu_InterruptEnable(IpiPtr, IPI_MASK_SELF);

    return XST_SUCCESS;
}

static int SetupInterruptSystem(XScuGic *GicPtr,
                                XIpiPsu *IpiPtr,
                                XIntc   *AxiIntcPtr)
{
    int Status;
    XScuGic_Config *GicCfgPtr;

    GicCfgPtr = XScuGic_LookupConfig(GIC_DEVICE_ID);
    if (GicCfgPtr == NULL) {
        return XST_FAILURE;
    }

    Status = XScuGic_CfgInitialize(GicPtr, GicCfgPtr,
                                   GicCfgPtr->CpuBaseAddress);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    /* Initialize exception table once */
    Xil_ExceptionInit();

    /* Connect IPI interrupt handler */
    Status = XScuGic_Connect(GicPtr, IPI_INT_ID,
                             (Xil_ExceptionHandler)IpiHandler,
                             (void *)IpiPtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    /* Connect AXI INTC fabric interrupt (cascaded controller) */
    Status = XScuGic_Connect(GicPtr, FABRIC_INTC_IRQ_ID,
                             (Xil_ExceptionHandler)XIntc_InterruptHandler,
                             (void *)AxiIntcPtr);
    if (Status != XST_SUCCESS) {
        return Status;
    }

    /* Enable both interrupt sources in GIC */
    XScuGic_Enable(GicPtr, IPI_INT_ID);
    XScuGic_Enable(GicPtr, FABRIC_INTC_IRQ_ID);

    /* Register GIC handler with the CPU exception system */
    Xil_ExceptionRegisterHandler(
        XIL_EXCEPTION_ID_IRQ_INT,               // <- use IRQ_INT consistently
        (Xil_ExceptionHandler)XScuGic_InterruptHandler,
        GicPtr
    );

    /* Globally enable interrupts */
    Xil_ExceptionEnable();

    return XST_SUCCESS;
}


int TestTmp117(void)
{
    u16 temp_bytes;
    int Status;
    double temperature;
    xil_printf("Testing TMP117 on AXI IIC (addr 0x4B)...\r\n");
    Status = platform_i2c_read(0x4B, 0x00, &temp_bytes);
    temperature = (double)temp_bytes * 7.8125e-3;
    int whole = temperature;
    int thousandths = (temperature - whole) * 1000;
    xil_printf("Temperature = %d.%03d\r\n", whole, thousandths);

    return Status;
}


void init_axi_gpio(void)
{
    int status = XGpio_Initialize(&Gpio_DDS_Chan, XPAR_AXI_GPIO_DDS_CHAN_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("GPIO DDS Chan - init failed\r\n");
        return;
    }

    status = XGpio_Initialize(&Gpio_Sample, XPAR_AXI_GPIO_SAMPLE_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("GPIO Sample - init failed\r\n");
        return;
    }

    status = XGpio_Initialize(&Gpio_Tui_Trigger, XPAR_AXI_GPIO_TUI_TRIGGER_BASEADDR);
    if (status != XST_SUCCESS) {
        xil_printf("GPIO Tui Trigger - init failed\r\n");
        return;
    }

    // Set channel as input: 1 = input, 0 = output
    XGpio_SetDataDirection(&Gpio_DDS_Chan, GPIO_CH, 0xFF);
    XGpio_SetDataDirection(&Gpio_Sample, GPIO_CH, 0x00);
    XGpio_SetDataDirection(&Gpio_Tui_Trigger, GPIO_CH, 0x00);

    // Enable AXI GPIO interrupts
    XGpio_InterruptEnable(&Gpio_DDS_Chan, XGPIO_IR_CH1_MASK);
    XGpio_InterruptGlobalEnable(&Gpio_DDS_Chan);
}


/************** main() **************/
int main(void)
{
    /* build NUM_PRESETS presets each with NUM_CHANNELS_RX aperture_channel_config_t entries */
    aperture_channel_config_t presets[NUM_PRESETS][NUM_CHANNELS];

    for (int p=0;p<NUM_PRESETS;++p) {
        for (int ch=0; ch<NUM_CHANNELS; ++ch) {
            presets[p][ch].tuning = 0;
            presets[p][ch].matching = 0;
            presets[p][ch].detune_enable = 0;
        }
    }

    int Status;

    xil_printf("R5 bring-up: PL IRQ + IPI\r\n");

    /* Map PL IO before touching 0xA0.. regs */
    Map_PlIo();

    init_axi_gpio();

   
    /***** Init IIC *****/
    Status = platform_i2c_init(XPAR_AXI_IIC_0_DEVICE_ID);
    if (Status != XST_SUCCESS) {
        xil_printf("InitAxiIic failed: %d\r\n", Status);
        return -7;
    }
    // TestTmp117();
    // TestMuxRead();

    /***** Init AXI INTC in PL *****/
    Status = XIntc_Initialize(&Intc, XPAR_XINTC_1_BASEADDR);
    if (Status != XST_SUCCESS) {
        xil_printf("XIntc init failed: %d\r\n", Status);
        return -1;
    }

    Status = XIntc_Connect(&Intc, 0,
                           (XInterruptHandler)DeviceDriverHandler,
                           NULL);
    if (Status != XST_SUCCESS) {
        xil_printf("XIntc connect failed: %d\r\n", Status);
        return -2;
    }

    XIntc_Enable(&Intc, 0);

    Status = XIntc_Start(&Intc, XIN_REAL_MODE);
    if (Status != XST_SUCCESS) {
        xil_printf("XIntc start failed: %d\r\n", Status);
        return -3;
    }

    /***** Init IPI *****/
    Status = InitIpi(&IpiInst);
    if (Status != XST_SUCCESS) {
        xil_printf("InitIpi failed: %d\r\n", Status);
        return -4;
    }


    /***** Setup GIC with BOTH interrupt sources *****/
    Status = SetupInterruptSystem(&GicInst, &IpiInst, &Intc);
    if (Status != XST_SUCCESS) {
        xil_printf("SetupInterruptSystem failed: %d\r\n", Status);
        return -5;
    }

    /***** Initialize shared memory value for IPI demo *****/
    ipi_msg_t *msg = (ipi_msg_t *)SHARED_MEM_ADDR;
    msg->magic  = IPI_MAGIC;
    msg->cmd    = 0;
    msg->len    = 0;
    msg->status = 0;
    for (int i = 0; i < IPI_MAX_PAYLOAD_BYTES; i++) {
        msg->payload[i] = 0;
    }
    Xil_DCacheFlushRange((UINTPTR)msg, sizeof(ipi_msg_t));

    atc_init();
    atc_precompute_sets(presets, NUM_PRESETS);

    int echos = 0;
    int scans = 0;
    int scans_target = 0;
    int t_end = 0;
    int test_seq = 0;


    // main loop 
    for (;;) {
        /* Apply new tuning set */
        if (aperture_tuning_change_flag) {
            aperture_tuning_change_flag = 0;
            uint8_t p = pending_preset;
            xil_printf("Aperture Tuning Change ----- (%d)\r\n", p);
            atc_apply_preset_blocking(p);
        }
        
        /* Pulse TUI Trigger input, not needed on initial set */
        if (pulse_tui_start_flag){   
            pulse_tui_start_flag = 0;       
            XGpio_DiscreteWrite(&Gpio_Tui_Trigger, GPIO_CH, 1);
            usleep(TUI_TRIGGER_US); 
            XGpio_DiscreteWrite(&Gpio_Tui_Trigger, GPIO_CH, 0);
        }

        if (test_seq){
            if (scans < scans_target){
                xil_printf("Scan %d\r\n", scans);
                int echos_scan = echos                
;                while (echos_scan > 0){
                    XGpio_DiscreteWrite(&Gpio_Sample, GPIO_CH, 1);
                    usleep(400); 
                    XGpio_DiscreteWrite(&Gpio_Sample, GPIO_CH, 0);
                    usleep(400); 
                    echos_scan--;                    
                }
                scans++;
            }
            if (scans == scans_target){
                test_seq = 0;
                scans_target = 0;
                xil_printf("Sample test sequence complete\r\n");
            }
            usleep(t_end); 
        }

        if (ipi_message_flag) {
            ipi_message_flag = 0;

            /* RX entire aperture tuning config for scan */
            if (ipi_buffer.cmd == 2){
                for (uint32_t i = 0; i < ipi_buffer.len && i < IPI_MAX_PAYLOAD_BYTES; i=i+3) {
                    // xil_printf("%02x) %02x %02x %02x\r\n",i, (unsigned int)ipi_buffer.payload[i], (unsigned int)ipi_buffer.payload[i+1], (unsigned int)ipi_buffer.payload[i+2]);
                    presets[i/3/NUM_PRESETS][(i/3)%(NUM_CHANNELS)].tuning = ipi_buffer.payload[i];
                    presets[i/3/NUM_PRESETS][(i/3)%(NUM_CHANNELS)].matching = ipi_buffer.payload[i+1];
                    presets[i/3/NUM_PRESETS][(i/3)%(NUM_CHANNELS)].detune_enable = ipi_buffer.payload[i+2];
                }

                atc_precompute_sets(presets, NUM_PRESETS);
                
                xil_printf("Tuning presets computed\r\n");

                pending_preset = 0;
                pulse_tui_start_flag = 0;
                aperture_tuning_change_flag = 1;

            }
            /* RX Tuning set change for Sequence Start, Triggers Tui 
               (usually triggered from GPIO interrupt from PL) */
            else if (ipi_buffer.cmd == 3){
                pending_preset = ipi_buffer.payload[0];
                pulse_tui_start_flag = 1;
                aperture_tuning_change_flag = 1;
                xil_printf("Setting Initial Tuning = %d\r\n", pending_preset);
            }

            else if (ipi_buffer.cmd == 4){
                if (test_seq == 0){
                    
                    uint32_t rxdata[3];

                    size_t n = ipi_buffer.len / 4;

                    for (size_t i = 0; i < n; i++) {
                        size_t j = 4 * i;
                        rxdata[i] =  (uint32_t)ipi_buffer.payload[j]
                                | ((uint32_t)ipi_buffer.payload[j + 1] << 8)
                                | ((uint32_t)ipi_buffer.payload[j + 2] << 16)
                                | ((uint32_t)ipi_buffer.payload[j + 3] << 24);
                    }

                    echos = rxdata[0];
                    scans_target = rxdata[1];
                    scans = 0;
                    t_end = rxdata[2];

                    xil_printf("Sample test sequence\r\n");
                    xil_printf("echoes: %d\r\n", echos);
                    xil_printf("scans:  %d\r\n", scans_target);
                    xil_printf("t_end:  %d\r\n", t_end);
                    test_seq = 1;
                }
            }

        }

        /* Check for work to be done before sleep in wait for interrupt */
        if ((aperture_tuning_change_flag == 0) && (ipi_message_flag == 0) && (test_seq == 0)) {
            __asm__ volatile("wfi");
        }
    }
    return 0;
}
