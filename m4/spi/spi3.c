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

    // SPI alternate function on GPIO5_IO11 (MOSI), GPIO5_IO10 (SCL)
    // no MISO and SS0 available
    IOMUXC_SetPinMux(IOMUXC_UART1_RXD_ECSPI3_SCLK, 0U);
    IOMUXC_SetPinMux(IOMUXC_UART1_TXD_ECSPI3_MOSI, 0U);
}

// TODO: currently not working
// there seems to be signal even with disabled uart1 on linux
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("SPI3 example started\r\n");

    CLOCK_EnableClock(kCLOCK_Ecspi3);
    CLOCK_SetRootMux(kCLOCK_RootEcspi3, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI3 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi3, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

    ecspi_master_config_t masterConfig;
    ECSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 50000U;

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi3)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi3));

    ECSPI_MasterInit(ECSPI3, &masterConfig, clk);

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
      ECSPI_MasterTransferBlocking(ECSPI3, &masterXfer);
      for(int i = 0; i < 240000; i++) asm("nop");
    }
}
