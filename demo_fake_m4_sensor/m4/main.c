#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_uart.h"
#include "data.h"

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMX8MQ_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE 0xB8000000
#define RPMSG_LITE_NS_ANNOUNCE_STRING "m4-channel"
#define LOCAL_EPT_ADDR (30)


TaskHandle_t measure_task;

volatile rpmsg_queue_handle my_queue;
struct rpmsg_lite_instance *volatile my_rpmsg;

volatile float sigma = 0.2;

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_UART3_RXD_UART3_RX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART3_RXD_UART3_RX, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
    IOMUXC_SetPinMux(IOMUXC_UART3_TXD_UART3_TX, 0U);
    IOMUXC_SetPinConfig(IOMUXC_UART3_TXD_UART3_TX, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
}

float gaussrand() {
  const int nsum = 25;

  float x = 0;
  int i;
  for(i = 0; i < nsum; i++) {
    x += (float) rand() / RAND_MAX;
  }

  x -= nsum / 2.0;
  x /= sqrt(nsum / 12.0);

  return x;
}

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data) {}

// task processess received commands from Linux side
void recv_cmd_task(void *param) {
  struct Command cmd;
  for(;;) {
    int ret = rpmsg_queue_recv(my_rpmsg, my_queue, NULL, (char *)&cmd, sizeof(cmd), NULL, RL_BLOCK);
    assert(ret == 0);

    if(cmd.type == CMD_PAUSE) {
      if(cmd.u8) {
        printf("Pausing measurements\r\n");
        vTaskSuspend(measure_task);
      } else {
        printf("Resuming measurements\r\n");
        vTaskResume(measure_task);
      }
    } else if(cmd.type == CMD_SIGMA) {
      sigma = cmd.f32; 
      printf("set new sigma error\r\n");
    } else {
      printf("Unknown cmd %d\r\n", cmd.type);
    }
  }
}

void app_task(void *param) {
    volatile unsigned long remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    // TODO: we are immediatelly ready
    my_rpmsg->link_state = 1;

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);
    
    if(xTaskCreate(recv_cmd_task, "RECV_CMD", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
      printf("could not create recv_cmd_task");
      for(;;);
    }
   
    // TODO: we are expecting that remote side has 0x400 rpmsg addr for now
    // so that we dont need to receive first message
    remote_addr = 0x400;
    uint32_t t = 0;
    struct Measurement m;
		float prev_pos = 0;
		float prev_velocity = 0;
    for(;;) {
      m.real_position = sin(t/10.0);

      // add some gauss noise to faked measured position
			float observed_pos = m.real_position + gaussrand() * sigma;

			m.position = observed_pos;
			m.velocity = observed_pos - prev_pos; 
      m.acceleration = m.velocity - prev_velocity;

      prev_pos = m.position;
      prev_velocity = m.velocity;
      t++;

      rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, (char *)&m, sizeof(m), RL_BLOCK);
      printf("sent %d\r\n", t);
      vTaskDelay(100);
    }

    for(;;);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    if(xTaskCreate(app_task, "APP_TASK", 256, NULL, tskIDLE_PRIORITY + 1, &measure_task) != pdPASS) {
        PRINTF("\r\nFailed to create application task\r\n");
        for(;;);
    }

    vTaskStartScheduler();

    PRINTF("Failed to start FreeRTOS on core0.\n");
    for(;;);
}
