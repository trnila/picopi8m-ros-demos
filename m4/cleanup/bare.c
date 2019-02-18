#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"

#define GI_STOP 1 /* 4th General interrupt */

__attribute__((weak)) void cleanup() {
    printf("m4 stop requested\r\n"); 
}

void MU_M4_IRQHandler() {
    if(MUB->SR & MU_SR_GIPn(GI_STOP)) {
        // clear pending ISR
        MUB->SR |= MU_SR_GIPn(GI_STOP);
        
        // disable all interrupts
        asm("cpsid i");

        // call user function for cleanup
        cleanup();

        // send completion to the linux
        MUB->CR |= MU_CR_GIRn(GI_STOP);
        for(;;);
    } else {
        printf("Unhandled MU\r\n");
        for(;;);
    }
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    // enable MU general interrupt
    MUB->CR |= MU_CR_GIEn(GI_STOP);
    NVIC_EnableIRQ(MU_M4_IRQn);

    printf("Hello world\r\n");
    for(;;);
}
