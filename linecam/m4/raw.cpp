#include <stdint.h>
#include "fsl_ecspi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

static int pixel;
static struct Bag *bag;

// because we are using c++, we have to make irq handler with C signature
extern "C" void ECSPI1_IRQHandler() {
  bag->frame[pixel++] = ECSPI1->RXDATA;
  if(pixel < bag->frame_size) {
    // make a clock pulse
    CLK_PORT->DR |= 1 << CLK_PIN;
    CLK_PORT->DR &= ~(1 << CLK_PIN);
    // send ADC channel 0
    ECSPI1->TXDATA = 0;
  } else {
    // wake publish task when we measured last pixel
    int err = xSemaphoreGiveFromISR(bag->publish_semaphore, NULL);
    assert(err == pdTRUE);
  }
}

void measure_task(void *param) {
  bag = (struct Bag*) param;

  // enable IRQ and set it up
  EnableIRQ(ECSPI1_IRQn);
  NVIC_SetPriority(ECSPI1_IRQn, 2);
  ECSPI_EnableInterrupts(ECSPI1, kECSPI_RxFifoReadyInterruptEnable | kECSPI_RxFifoOverFlowInterruptEnable);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  for(;;) {
    xSemaphoreTake(bag->measure_semaphore, portMAX_DELAY);
    CLK_set(0);
    SI_set(1);
    CLK_set(1);
    SI_set(0);
    CLK_set(0);

    pixel = 0;

    // write ADC channel to transmit register
    // it will be pushed to the TXFIFO and with current settings immediately sent over SPI
    ECSPI1->TXDATA = 0;

    vTaskDelayUntil(&xLastWakeTime, 10);
  }
}
