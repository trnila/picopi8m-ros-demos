#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"

#define SIZE 16

uint32_t tx[SIZE];
uint32_t rx[SIZE];

void BOARD_InitPins(void) {
    // SPI alternate function on GPIO5_IO11 (MOSI), GPIO5_IO10 (SCL)
    // no MISO and SS0 available
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_MOSI_ECSPI2_MOSI, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_SCLK_ECSPI2_SCLK, 0U);
}

// this example sends first 16 letters from alphabet on ECSPI2_MOSI in loop
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("SPI2 example started\r\n");

    CLOCK_EnableClock(kCLOCK_Ecspi2);
    CLOCK_SetRootMux(kCLOCK_RootEcspi2, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi2, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 50000U;

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi2)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi2));

    ECSPI_MasterInit(ECSPI2, &masterConfig, clk);

    for(int i = 0; i < SIZE; i++) {
      tx[i] = 'a' + i;
    }

    for(;;) {
      rx[0] = 0;
      ecspi_transfer_t masterXfer;
      masterXfer.txData = tx;
      masterXfer.rxData = rx;
      masterXfer.dataSize = SIZE;
      masterXfer.channel = kECSPI_Channel0;
      ECSPI_MasterTransferBlocking(ECSPI2, &masterXfer);
      for(int i = 0; i < 240000; i++) asm("nop");
    }
}
