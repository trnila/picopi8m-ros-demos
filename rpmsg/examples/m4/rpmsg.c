#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "rsc_table_rpmsg.h"

static char app_buf[512];

void app_task(void *param) {
  int result;
  struct rpmsg_lite_instance *my_rpmsg = create_rpmsg_from_resources();

  rpmsg_queue_handle my_queue = rpmsg_queue_create(my_rpmsg);
  struct rpmsg_lite_endpoint* my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
  rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_CHANNEL, RL_NS_CREATE);

  printf("Announced channel " RPMSG_CHANNEL "\r\n");
  for (;;) {
    unsigned long remote_addr;

    int rx_len;
    char *rx_buf;
    result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, &remote_addr, &rx_buf, &rx_len, RL_BLOCK);
    assert(result == RL_SUCCESS);
    assert(rx_len < sizeof(app_buf));

    memcpy(app_buf, rx_buf, rx_len);
    app_buf[rx_len] = 0;

    printf("received: \"%s\" [len : %d]\r\n", app_buf, rx_len);

    /* Get tx buffer from RPMsg */
    unsigned long tx_len;
    char* tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &tx_len, RL_BLOCK);
    assert(tx_buf);
    /* Copy string to RPMsg tx buffer */
    memcpy(tx_buf, app_buf, tx_len);

    /* Echo back received message with nocopy send */
    result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, tx_len);
    assert(result == RL_SUCCESS);

    /* Release held RPMsg rx buffer */
    result = rpmsg_queue_nocopy_free(my_rpmsg, rx_buf);
    assert(result == RL_SUCCESS);
  }
}

int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();  

  if (xTaskCreate(app_task, "APP_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    printf("\r\nFailed to create application task\r\n");
    for(;;);
  }

  vTaskStartScheduler();
  for(;;);
}
