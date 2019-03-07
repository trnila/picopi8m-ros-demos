#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"

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

}

/**
 * this example requests each channel separately, so chip select is deactivated between channels
*/
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

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
    for(;;);

    for(;;) {
        for(int chan = 0; chan < 8; chan++) {
            uint32_t rx = 0;
            uint32_t tx = ((chan + 1) & 0x7) << (3 + 8);
            ecspi_transfer_t masterXfer;
            masterXfer.txData = &tx;
            masterXfer.rxData = &rx;
            masterXfer.dataSize = 1;
            masterXfer.channel = kECSPI_Channel0;
            ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);
            printf("%5d ", rx);
        }
        printf("\r\n");

        for(int i = 0; i < 240000; i++) asm("nop");
    }
}
