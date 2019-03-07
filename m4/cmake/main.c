#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_uart.h"


#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel-1"
static char buff[512];

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data) {}

const int pins[] = {
  145, 144, 139, 138, 65, 131, 135, 136, 134, 147, 134, 147, 79, 81, 130, 120, 67,
  /*155, 154,*/ 121, 68, 64, 69, 137, 66, 146, 132, 80, 119, 122
};

GPIO_Type *gpios[] = GPIO_BASE_PTRS;

int is_pin_allowed(int pin) {
  for(int i = 0; i < sizeof(pins) / sizeof(*pins); i++) {
    if(pins[i] == pin) {
      return 1;
    }
  }
  return 0;
}

void pin_init(int pin) {
  gpio_pin_config_t conf;
  conf.direction = kGPIO_DigitalOutput;
  conf.outputLogic = 0;
  conf.interruptMode = kGPIO_NoIntmode;

  GPIO_PinInit(gpios[pin/32 + 1], pin % 32, &conf);
}

void pin_set(int pin, int state) { 
  GPIO_PinWrite(gpios[pin/32 + 1], pin % 32, state);
}

void send_back(struct rpmsg_lite_instance *my_rpmsg, struct rpmsg_lite_endpoint *my_ept, unsigned long remote_addr, const char *fmt, ...) {
  unsigned long size;
  char* tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
  assert(tx_buf);

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(tx_buf, size, fmt, args);
  va_end(args);

  printf("%s", tx_buf);

  if(rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len) != 0) {
    printf("\r\nsend failed\r\n");
    for(;;);
  }
}

void app_task(void *param) {
    volatile unsigned long remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;
    volatile rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *volatile my_rpmsg;
    char *rx_buf;
    int len;
    int result;

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    my_rpmsg->link_state = 1;

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    printf("Ready\r\n");
    int offset = 0;
    for (;;) {
        result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (unsigned long *)&remote_addr, &rx_buf, &len, RL_BLOCK);
        if (result != 0) {
          printf("could not recv\r\n");
          for(;;);
        }
        
        for(int i = 0; i < len; i++) {
          if(rx_buf[i] == '\n' || rx_buf[i] == '\r') {
            if(offset <= 0) {
              continue;
            }

            buff[offset] = 0;
            offset = 0;
            char* action = buff;
            char* arg = strchr(buff, ' ');
            if(!arg) {
              send_back(my_rpmsg, my_ept, remote_addr, "no argument found: '%s'\r\n", buff);
              continue;
            }

            char *end;
            int pin = strtol(arg, &end, 10);
            if(arg + strlen(arg) != end) {
              send_back(my_rpmsg, my_ept, remote_addr, "invalid argument: '%s'\r\n", arg);
              continue;
            }

            if(!is_pin_allowed(pin)) {
              send_back(my_rpmsg, my_ept, remote_addr, "pin %d not allowed\r\n", pin);
              continue;
            }

            if(strncmp("out", action, 3) == 0) {
              pin_init(pin);
              send_back(my_rpmsg, my_ept, remote_addr, "pin %d configured\r\n", pin);
            } else if(strncmp("set", action, 3) == 0) {
              pin_set(pin, 1);
              send_back(my_rpmsg, my_ept, remote_addr, "pin %d set\r\n", pin);
            } else if(strncmp("clr", action, 3) == 0) {
              pin_set(pin, 0);
              send_back(my_rpmsg, my_ept, remote_addr, "pin %d cleared\r\n", pin);
            } else {
              send_back(my_rpmsg, my_ept, remote_addr, "unknown action '%s'\r\n", action);
            }
          } else if(rx_buf[i] != '\r') {
            buff[offset++] = rx_buf[i];
          }
          //printf("recv: %d\r\n", rx_buf[i]);
        }

        result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
        if (result != 0) {
            printf("Could not free buffer\r\n");
            for(;;);
        }
    }
}


int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    uart_config_t config;
    UART_GetDefaultConfig(&config);
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;

#define BOARD_DEBUG_UART_CLK_FREQ                                                          \
        CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootUart4)) / \
            (CLOCK_GetRootPostDivider(kCLOCK_RootUart4)) / 10
    UART_Init(UART4, &config, BOARD_DEBUG_UART_CLK_FREQ);

    uint8_t txbuff[] = "Uart polling example\r\nBoard will send back received characters\r\n";
    for(;;) {
      printf("w\r\n");
      UART_WriteBlocking(UART4, txbuff, sizeof(txbuff) - 1);
    }

    if (xTaskCreate(app_task, "app", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        PRINTF("\r\nFailed to create application task\r\n");
        for(;;);
    }

    vTaskStartScheduler();
    PRINTF("Failed to start FreeRTOS.\n");
    for(;;);
}
