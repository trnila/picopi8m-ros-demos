#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "fsl_ecspi_freertos.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "common.h"

void app_task(void *param) {
    CLOCK_EnableClock(kCLOCK_Ecspi1);
    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 8000000;
    masterConfig.burstLength = 16 * 8 /* 2 bytes * channels */;
    masterConfig.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
    masterConfig.channelConfig.chipSlectActiveState = kECSPI_ChipSelectActiveStateLow;
    masterConfig.channelConfig.waveForm = kECSPI_WaveFormSingle;
    masterConfig.channelConfig.polarity = kECSPI_PolarityActiveLow;
    masterConfig.channelConfig.phase = kECSPI_ClockPhaseSecondEdge;

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));

    // important for freertos
    NVIC_SetPriority(ECSPI1_IRQn, 2);

    ecspi_rtos_handle_t h;
    status_t err = ECSPI_RTOS_Init(&h, ECSPI1, &masterConfig, clk);
    assert(err == kStatus_Success);

    // prepare transmit buffer with channels:
    uint32_t tx[] = {
        SAMPLE(1, 2),
        SAMPLE(3, 4),
        SAMPLE(5, 6),
        SAMPLE(7, 0),
    };
    int len = sizeof(tx) / sizeof(*tx);
    uint32_t rx[len];

    for(;;) {
	ecspi_transfer_t xfer;
	xfer.txData = tx;
	xfer.rxData = rx;
	xfer.dataSize = len;
	xfer.channel = kECSPI_Channel0;

	err = ECSPI_RTOS_Transfer(&h, &xfer);
	assert(err == kStatus_Success);

        for(int i = 0; i < len; i++) {
            printf("%5d %5d ", SAMPLE_A(rx[i]), SAMPLE_B(rx[i]));
        }
	printf("\r\n");

	vTaskDelay(10);
    }
}

/**
 * example show how to read all channels in single SPI transaction without busy waiting,
 * so core can processing something else while transmiting SPI 
 */
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    if (xTaskCreate(app_task, "app", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
	printf("\r\nFailed to create application task\r\n");
	for(;;);
    }

    vTaskStartScheduler();
    printf("Failed to start FreeRTOS.\n");
    for(;;);
}
