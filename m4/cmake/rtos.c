#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"

void app_task(void *param) {
  printf("hello world\r\n");
  for(;;);
}


int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    if (xTaskCreate(app_task, "app", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create application task\r\n");
        for(;;);
    }

    vTaskStartScheduler();
    printf("Failed to start FreeRTOS.\n");
    for(;;);
}
