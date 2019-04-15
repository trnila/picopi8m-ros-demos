#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"

void do_hardfault();

int d(int x) {
   printf("%d\n", x);
   do_hardfault();
}

int c(int x) {
   d(2 * x);
}


int b(int x) {
   c(2 * x);
}

int a(int x) {
   b(2 * x);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    SCB->CFSR = 0x2000000;
    SCB->CCR |= 0x10;

    printf("Started\r\n");
    a(1);
    printf("FAIL\r\n");
    for(;;);
}
