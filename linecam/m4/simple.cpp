#include <stdint.h>
#include "fsl_ecspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

uint16_t adc_read(int chan) {
    uint32_t rx = 0;
    uint32_t tx = ((chan) & 0x7) << (3 + 8);
    ecspi_transfer_t masterXfer;
    masterXfer.txData = &tx;
    masterXfer.rxData = &rx;
    masterXfer.dataSize = 1;
    masterXfer.channel = kECSPI_Channel0;
    ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);
    return rx;
}


void measure_task(void* param) {
    struct Bag *bag = (struct Bag*) param;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        xSemaphoreTake(bag->measure_semaphore, portMAX_DELAY);

        CLK_set(0);
        SI_set(1);
        CLK_set(1);
        SI_set(0);

        for(int i = 0; i < CAMERA_POINTS; i++) {
            CLK_set(0);
            bag->frame[i] = adc_read(0);
            CLK_set(1);
        }
        xSemaphoreGive(bag->publish_semaphore);

        vTaskDelayUntil(&xLastWakeTime, 10);
    }
}
