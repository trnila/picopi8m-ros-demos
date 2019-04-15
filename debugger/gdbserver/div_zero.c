#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"


int division(int b, int a) {
   int d = 15;
   return (d + a) / b;
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    SCB->CFSR = 0x2000000;
    SCB->CCR |= 0x10;

    printf("Started\r\n");
    int val = 0;
    volatile int x = division(val, 128);
    printf("FAIL\r\n");
    for(;;);
}
