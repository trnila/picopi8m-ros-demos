#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("Hello world\r\n");
    for(;;);
}
