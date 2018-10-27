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

    // set UART4 alternate function for GPIO5_IO11 and GPIO5_IO10
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_MOSI_UART4_TX, 0U);
    IOMUXC_SetPinMux(IOMUXC_ECSPI2_SCLK_UART4_RX, 0U);
}


#define RING_SIZE 128 // must be power of 2
uint8_t ring[RING_SIZE];
volatile int head;
volatile int tail;

void UART4_IRQHandler() {
  if(UART_GetStatusFlag(UART4, kUART_RxDataReadyFlag)) {
    ring[tail] = UART_ReadByte(UART4);
    tail = (tail + 1) & (RING_SIZE - 1);
  }
}

// this example demonstrates how to use interrupt to receive characters on UART4
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

    uint8_t tx[] = "Started, now type some characters\r\n";
    UART_WriteBlocking(UART4, tx, sizeof(tx) - 1);

    UART_EnableInterrupts(UART4, kUART_RxDataReadyEnable);
    EnableIRQ(UART4_IRQn);

    for(;;) {
      if(head != tail) {
        printf("got: '%c'\r\n", ring[head]);
        head = (head + 1) & (RING_SIZE - 1);
      }
    }
}
