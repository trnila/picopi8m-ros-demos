#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

QueueHandle_t Q;

// place addresses that are dereferenced by consumer task 
void producer_task(void *arg) {
    int a = 1, b = 2, c = 3;
    int *items[] = {
        &a, &b, &c, 0x40000000 /* DDR memory, that is not accessible */
    };
    for(;;) {
        for(int i = 0; i < sizeof(items) / sizeof(*items); i++) {
            xQueueSend(Q, items + i, portMAX_DELAY);
            printf("placed %lx\r\n", items[i]);
        }        
    }
}

void consumer_task(void *arg) {
    for(;;) {
        uint32_t* v;
        xQueueReceive(Q, &v, portMAX_DELAY);
        printf("read: %d\r\n", *v);
    }
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    Q = xQueueCreate(2, sizeof(uint32_t*));
    if(Q == NULL) {
        printf("Failed to create queue\r\n");
        for(;;);
    }

    if (xTaskCreate(producer_task, "APP_TASK", 512, Q, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create application task\r\n"); 
        for(;;);
    }
    if (xTaskCreate(consumer_task, "APP_TASK", 512, Q, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create application task\r\n"); 
        for(;;);
    }

    vTaskStartScheduler();
    for(;;);
}
