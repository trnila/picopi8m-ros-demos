#include <stdint.h>
#include "fsl_ecspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

struct TransferBag {
  struct Bag *bag;
  ecspi_transfer_t xfer;
  int pixel;
};

void ecspi_done_callback(ECSPI_Type *base, ecspi_master_handle_t *handle, status_t status, void *data) {
  assert(kStatus_Success == status);

  struct TransferBag *transfer_bag = (struct TransferBag*) data;

  transfer_bag->bag->frame[transfer_bag->pixel++] = *(handle->rxData - 1); // get value before increasing buffer
  if(transfer_bag->pixel < transfer_bag->bag->frame_size) {
    ECSPI_MasterTransferNonBlocking(ECSPI1, handle, &transfer_bag->xfer);
    CLK_set(1);
    CLK_set(0);
  } else {
    // last pixel, so wake publish task
    int err = xSemaphoreGiveFromISR(transfer_bag->bag->publish_semaphore, NULL);
    assert(err == pdTRUE);
  }
}

void measure_task(void *param) {
  struct Bag *bag = (struct Bag*) param;

  // FreeRTOS requires to set priority for interrupts
  NVIC_SetPriority(ECSPI1_IRQn, 2);

  // prepare bag that is used for passing structs to interrupts
  struct TransferBag transfer_bag{};
  transfer_bag.bag = bag;
  // we cant directly write to our frame array because it is uint16_t
  // driver requires to write to the uint32_t
  uint32_t rx, tx = 0;
  transfer_bag.xfer.rxData = &rx;
  transfer_bag.xfer.txData = &tx;
  transfer_bag.xfer.dataSize = 1;
  transfer_bag.xfer.channel = kECSPI_Channel0;

  // create handle for nonblocking SPI transfers
  ecspi_master_handle_t h;
  ECSPI_MasterTransferCreateHandle(ECSPI1, &h, ecspi_done_callback, &transfer_bag); 

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    xSemaphoreTake(bag->measure_semaphore, portMAX_DELAY);

    CLK_set(0);
    SI_set(1);
    CLK_set(1);
    SI_set(0);
    CLK_set(0);

    transfer_bag.pixel = 0;
    ECSPI_MasterTransferNonBlocking(ECSPI1, &h, &transfer_bag.xfer);
    // in last pixel interrupt we will wake task by increasing semaphore publish_semaphore

    vTaskDelayUntil(&xLastWakeTime, 10);
  }
}
