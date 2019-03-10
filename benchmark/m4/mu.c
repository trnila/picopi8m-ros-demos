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

void MU_M4_IRQHandler() {
	int data = MU_ReceiveMsgNonBlocking(MUB, 0);
//	printf("irqed %d\r\n", data);
	MU_SendMsg(MUB, 0, data + 1);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();
    PRINTF("mu bench starting...\r\n");

    MU_Init(MUB);
    MU_EnableInterrupts(MUB, (1U << 27U));
    NVIC_EnableIRQ(MU_M4_IRQn);

    PRINTF("waiting for kernel ping\r\n");
    for(;;);
}
