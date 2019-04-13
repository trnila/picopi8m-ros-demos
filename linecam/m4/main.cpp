#include "main.h"
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

ros::NodeHandle nh;

uint16_t frame[CAMERA_POINTS];
std_msgs::UInt16MultiArray container;
ros::Publisher measurements("/linecam0", &container);
struct Bag bag;

void measure_task(void*);


void BOARD_InitPins(void) {
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

void rosserial_task(void *param) {
    for(;;) {
        nh.spinOnce();
    }
}

void ros_task(void *param) {
    struct Bag *bag = (struct Bag*) param;

    nh.advertise(measurements);

    container.data = bag->frame;
    container.data_length = bag->frame_size;

    for(;;) {
        xSemaphoreTake(bag->publish_semaphore, portMAX_DELAY);
        line_print(frame);
        measurements.publish(&container);
        
        xSemaphoreGive(bag->measure_semaphore);
    }
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    // setup SI and CLK pin as an output pin
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(SI_PORT, SI_PIN, &conf);
    GPIO_PinInit(CLK_PORT, CLK_PIN, &conf);

    // enable clocks for SPI and setup SPI parameters
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

    // prepare semaphores
    bag.publish_semaphore = xSemaphoreCreateBinary();
    assert(bag.publish_semaphore != NULL);
    bag.measure_semaphore = xSemaphoreCreateBinary();
    assert(bag.measure_semaphore != NULL);

    // increase measure semaphore so we will start with measure task
    BaseType_t ret = xSemaphoreGive(bag.measure_semaphore);
    assert(ret == pdTRUE);

    bag.frame = frame;
    bag.frame_size = CAMERA_POINTS; 

    // initialize ros node
    nh.initNode();

    if(xTaskCreate(rosserial_task, "ROSSERIAL_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create task\r\n");
        for(;;);
    }

    if (xTaskCreate(measure_task, "MEASURE_TASK", 512, &bag, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create measure task\r\n"); 
        for(;;);
    }

    if (xTaskCreate(ros_task, "ROS_TASK", 512, &bag, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create ros task\r\n"); 
        for(;;);
    }

    vTaskStartScheduler();
    printf("Failed to start FreeRTOS\n");
    for(;;);
}
