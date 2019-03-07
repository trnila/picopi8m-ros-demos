#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_uart.h"
#include "pin_mux.h"
#include "clock_config.h"

void BOARD_InitPins(void) {
    // set UART4 alternate function for GPIO5_IO11 and GPIO5_IO10
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_MOSI_UART4_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_SCLK_UART4_RX, 0U);
}


int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("UART4 example started\r\n");
    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;

    CLOCK_SetRootMux(kCLOCK_RootUart4, kCLOCK_UartRootmuxSysPll1Div10); /* Set UART source to SysPLL1 Div10 80MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootUart4, 1U, 1U);                  /* Set root clock to 80MHZ/ 1= 80MHZ */   
    
    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootUart4)) / (CLOCK_GetRootPostDivider(kCLOCK_RootUart4)) / 10 /* div10 */;
    UART_Init(UART4, &config, clk);

    uint8_t tx[] = "Hello world\r\n";
    UART_WriteBlocking(UART4, tx, sizeof(tx) - 1);
    for(;;) {
      UART_ReadBlocking(UART4, tx, 1);

      if(tx[0] >= 'a' && tx[0] <= 'z') {
        tx[0] -= 'a' - 'A';
      }

      UART_WriteBlocking(UART4, tx, 1);
    }
}
