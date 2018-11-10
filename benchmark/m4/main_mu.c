#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_uart.h"
#include "fsl_mu.h"

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
}

void MU_M4_IRQHandler() {
	int data = MU_ReceiveMsgNonBlocking(MUB, 0);
//	printf("irqed %d\r\n", data);
	MU_SendMsg(MUB, 0, data + 1);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();
    PRINTF("mu bench starting...\r\n");

    MU_Init(MUB);
    MU_EnableInterrupts(MUB, (1U << 27U));
    NVIC_EnableIRQ(MU_M4_IRQn);
    MU_SendMsg(MUB, 0, 1);

    PRINTF("ready\r\n");
    for(;;);
}
