#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"

#define SIZE 16

uint8_t tx[SIZE];
uint8_t rx[SIZE];

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

// this example sends first 16 letters on MOSI and then checks if they were received on MISO,
// so you should connect them or use masterConfig->enableLoopback = 1
// chip select is actived on whole transaction
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("SPI1 with hardware chip-select (hw cs) for whole transaction example started\r\n");

    CLOCK_EnableClock(kCLOCK_Ecspi1);
    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 500000U;
    masterConfig.burstLength = 32 * SIZE / sizeof(uint32_t);

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));

    ECSPI_MasterInit(ECSPI1, &masterConfig, clk);

    // we need to rearrange 8 bits in 32 bits (change endianity of 32 bit words)
    // because bit on the highest address (of 32 bit) is sent over SPI bus as first
    // |    32 bits    |    32 bits    |    |    32 bits    |    32 bits    |
    // | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | => | 3 | 2 | 1 | 0 | 7 | 6 | 3 | 4 |
    for(int i = 0; i < SIZE; i++) {
      int pos = 4 * (i / 4) + 3 - (i & 3);

      tx[pos] = 'a' + i;
    }

    for(;;) {
      memset(rx, 0, SIZE * sizeof(*rx));
      ecspi_transfer_t masterXfer;
      masterXfer.txData = tx;
      masterXfer.rxData = rx;
      masterXfer.dataSize = SIZE / sizeof(uint32_t); // we are sending whole 32 bits
      masterXfer.channel = kECSPI_Channel0;
      ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);

      for(int i = 0; i < SIZE; i++) {
        int pos = 4 * (i / 4) + 3 - (i & 3);
        if(rx[pos] != 'a' + i) {
          printf("received %d but %d expected\r\n", rx[i], 'a' + i);
        }
      }

      for(int i = 0; i < 240000; i++) asm("nop");
    }
}
