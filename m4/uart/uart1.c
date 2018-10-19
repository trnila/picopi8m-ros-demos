#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_uart.h"
#include "pin_mux.h"
#include "clock_config.h"

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

    // set UART1 alternate function for GPIO5_IO22 and GPIO5_IO23
    // will break your linux console!
    // you should disable console in device tree
    // &uart1 {
    //    status = "disabled";
    // }
    IOMUXC_SetPinMux(IOMUXC_UART1_RXD_UART1_RX, 0U);
    IOMUXC_SetPinMux(IOMUXC_UART1_TXD_UART1_TX, 0U);
}


int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("UART1 example started\r\n");
    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;

    CLOCK_SetRootMux(kCLOCK_RootUart1, kCLOCK_UartRootmuxSysPll1Div10); /* Set UART source to SysPLL1 Div10 80MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootUart1, 1U, 1U);                  /* Set root clock to 80MHZ/ 1= 80MHZ */   
    
    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootUart1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootUart1)) / 10 /* div10 */;
    UART_Init(UART1, &config, clk);

    uint8_t tx[] = "Hello world\r\n";
    UART_WriteBlocking(UART1, tx, sizeof(tx) - 1);
    for(;;) {
      UART_ReadBlocking(UART1, tx, 1);

      if(tx[0] >= 'a' && tx[0] <= 'z') {
        tx[0] -= 'a' - 'A';
      }

      UART_WriteBlocking(UART1, tx, 1);
    }
}
