#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    int i = 0;

    for(;;) {
        printf("Hello world %d\r\n", i++);
        for(int i = 0; i < 6400000; i++) asm("nop");
    }
}
