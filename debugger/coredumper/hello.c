#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"

volatile int aa = 18;
volatile float bb;

char text[] = "hello world";

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    aa = 23;
    bb = 3.14159265359;
    text[0] = 'H';

    printf("Hello world\r\n");
    for(;;);
}
