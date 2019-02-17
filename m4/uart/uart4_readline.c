#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_uart.h"
#include "pin_mux.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "clock_config.h"

#define MAX_LINE_SIZE 32
#define MAX_LINES 8

// buffers for each line
char buffers[MAX_LINES][MAX_LINE_SIZE];

// in available queue are empty buffers
// in used are received lines ready to be consumed in user task
QueueHandle_t available, used;

// currently used buffer 
char *current_buffer;
int current_buffer_pos;


void BOARD_InitPins(void) {
  // set UART4 alternate function for GPIO5_IO11 and GPIO5_IO10
  IOMUXC_SetPinMux(IOMUXC_ECSPI2_MOSI_UART4_TX, 0U);
  IOMUXC_SetPinMux(IOMUXC_ECSPI2_SCLK_UART4_RX, 0U);
}

void UART4_IRQHandler() {
  if(!current_buffer) {
      // receive another free buffer
      xQueueReceiveFromISR(available, &current_buffer, NULL);
  }

  if(UART_GetStatusFlag(UART4, kUART_RxDataReadyFlag) && current_buffer) {
    uint8_t rcvd = UART_ReadByte(UART4);
    if(rcvd == '\n') {
      // get rid of \r
      if(current_buffer_pos > 0 && current_buffer[current_buffer_pos - 1] == '\r') {
        current_buffer_pos -= 1;
      }

      current_buffer[current_buffer_pos] = 0;

      // send received line to user task
      BaseType_t err = xQueueSendFromISR(used, &current_buffer, NULL);
      assert(err = pdTRUE);

      current_buffer = NULL;
      current_buffer_pos = 0;
    } else {
      current_buffer[current_buffer_pos++] = rcvd;

      if(current_buffer_pos >= MAX_LINE_SIZE) {
        //line too long, drop it
        current_buffer_pos = 0;        
      }
    }
  }
}

void app_task(void *param) {
  for(;;) {
    char *line;
    BaseType_t err = xQueueReceive(used, &line, portMAX_DELAY);
    assert(err == pdTRUE);

    int len = strlen(line);

    printf("Received line: '%s' (%p)\r\n", line, line);

    for(int i = 0; i < len; i++) {
      if(line[i] >= 'a' && line[i] <= 'z') {
        line[i] -= 'a' - 'A';
      }
    }
    line[len++] = '\n';

    // hold buffer and send response in blocking way
    UART_WriteBlocking(UART4, line, len);

    // return buffer to irq
    err = xQueueSend(available, &line, portMAX_DELAY);
    assert(err == pdTRUE);
  }
}

/**
 * example demonstrates how to receive lines in nonblocking way
 */
int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("UART4 example started\r\n");
  available = xQueueCreate(MAX_LINES, sizeof(char*));
  assert(available);
  used = xQueueCreate(MAX_LINES, sizeof(char*));
  assert(used);

  // put all buffers to available
  for(int i = 0; i < MAX_LINES; i++) {
    char* p = buffers[i];

    BaseType_t err = xQueueSend(available, &p, portMAX_DELAY);
    assert(err == pdTRUE);
  }

  uart_config_t config;
  UART_GetDefaultConfig(&config);
  config.baudRate_Bps = 115200U;
  config.enableTx = true;
  config.enableRx = true;

  CLOCK_SetRootMux(kCLOCK_RootUart4, kCLOCK_UartRootmuxSysPll1Div10); /* Set UART source to SysPLL1 Div10 80MHZ */
  CLOCK_SetRootDivider(kCLOCK_RootUart4, 1U, 1U);                  /* Set root clock to 80MHZ/ 1= 80MHZ */   

  uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootUart4)) / (CLOCK_GetRootPostDivider(kCLOCK_RootUart4)) / 10 /* div10 */;
  UART_Init(UART4, &config, clk);

  UART_EnableInterrupts(UART4, kUART_RxDataReadyEnable);
  NVIC_SetPriority(UART4_IRQn, 2);
  EnableIRQ(UART4_IRQn);

  if (xTaskCreate(app_task, "app", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    printf("\r\nFailed to create application task\r\n");
    for(;;);
  }

  vTaskStartScheduler();
  printf("Failed to start FreeRTOS.\n");
  for(;;);
}
