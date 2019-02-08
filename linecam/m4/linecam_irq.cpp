#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "task.h"
#include "semphr.h"

#define PRINT_OUTPUT 0
#define CAMERA_POINTS 128

#define SI_PORT GPIO4
#define SI_PIN 23

#define CLK_PORT GPIO4
#define CLK_PIN 26


ros::NodeHandle nh;

uint16_t frame[CAMERA_POINTS];
std_msgs::UInt16MultiArray container;
ros::Publisher measurements("/linecam0", &container);

void SI_set(int state) {
    GPIO_PinWrite(SI_PORT, SI_PIN, state);
}

void CLK_set(int state) {
    GPIO_PinWrite(CLK_PORT, CLK_PIN, state);
}

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_UART3_RXD_UART3_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART3_RXD_UART3_RX, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
            IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
    IOMUXC_SetPinMux(IOMUXC_UART3_TXD_UART3_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART3_TXD_UART3_TX, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
            IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);

    // SPI alternate function
    // ECSPI1_MOSI - GPIO5_IO07
    // ECSPI1_MISO - GPIO5_IO08
    // ECSPI1_CLK  - GPIO5_IO6
    // ECSPI1_SS0  - GPIO5_IO9
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MISO_ECSPI1_MISO, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MOSI_ECSPI1_MOSI, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_SCLK_ECSPI1_SCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_SS0_ECSPI1_SS0, 0U);

    IOMUXC_SetPinMux(IOMUXC_SAI2_RXD0_GPIO4_IO23, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI2_RXD0_GPIO4_IO23, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI2_TXD0_GPIO4_IO26, 
            IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
            IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
}

uint16_t adc_read(int chan) {
    uint32_t rx = 0;
    uint32_t tx = ((chan) & 0x7) << (3 + 8);
    ecspi_transfer_t masterXfer;
    masterXfer.txData = &tx;
    masterXfer.rxData = &rx;
    masterXfer.dataSize = 1;
    masterXfer.channel = kECSPI_Channel0;
    ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);
    return rx;
}

char to_color(uint16_t val) {
    const char table[] = ".,;!vlLFE$";
    return table[val / 409];
}

uint32_t tx = 0;
uint32_t rx;
int pixel;
ecspi_transfer_t masterXfer;
ecspi_master_handle_t h;

SemaphoreHandle_t sem;

void cb(ECSPI_Type *base, ecspi_master_handle_t *handle, status_t status, void *data) {
    frame[pixel] = rx;

    pixel++;
    if(pixel < 128) {
        ECSPI_MasterTransferNonBlocking(ECSPI1, &h, &masterXfer);
        CLK_set(1);
        CLK_set(0);
    } else {
        int err = xSemaphoreGiveFromISR(sem, NULL);
    }
}

void app_task(void *param) {
    NVIC_SetPriority(ECSPI1_IRQn, 2);

    nh.initNode();
    nh.advertise(measurements);

    container.data = frame;
    container.data_length = CAMERA_POINTS;

    sem = xSemaphoreCreateBinary();
    assert(sem != NULL);

    ECSPI_MasterTransferCreateHandle(ECSPI1, &h, cb, NULL); 

    masterXfer.txData = &tx;
    masterXfer.rxData = &rx;
    masterXfer.dataSize = 1;
    masterXfer.channel = kECSPI_Channel0;


    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        CLK_set(0);
        SI_set(1);
        CLK_set(1);
        SI_set(0);

        CLK_set(0);

        pixel = 0;
        ECSPI_MasterTransferNonBlocking(ECSPI1, &h, &masterXfer);
        xSemaphoreTake(sem, portMAX_DELAY);

#if PRINT_OUTPUT
        for(int i = 0; i < CAMERA_POINTS; i++) {
            printf("%c", to_color(frame[i]));
        }
        printf("\r\n");
#endif

        measurements.publish(&container);
        nh.spinOnce();

        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(SI_PORT, SI_PIN, &conf);
    GPIO_PinInit(CLK_PORT, CLK_PIN, &conf);

    CLOCK_EnableClock(kCLOCK_Ecspi1);
    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 12000000U;
    masterConfig.burstLength = 16;
    masterConfig.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));
    ECSPI_MasterInit(ECSPI1, &masterConfig, clk);


    if (xTaskCreate(app_task, "APP_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create application task\r\n"); 
        for(;;);
    }

    vTaskStartScheduler();
    printf("Failed to start FreeRTOS\n");
    for(;;);
}
