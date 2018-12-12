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
#include "fsl_uart.h"

#define RPMSG_LITE_SHMEM_BASE (0xB8000000U)
#define RPMSG_LITE_NS_ANNOUNCE_STRING "udp"

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

void app_task(void *param) {
    struct rpmsg_lite_endpoint *my_ept;
    struct rpmsg_lite_instance *my_rpmsg;

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    my_rpmsg->link_state = 1;

    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, NULL);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);

    int  i = 0;
    for (;;) {
        // get tx buffer
        unsigned long size;
        uint8_t *tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
        assert(tx_buf);

        // write directly to the tx buffer
        snprintf(tx_buf, size, "hello %d", i++);

        // send tx buffer
        int result = rpmsg_lite_send_nocopy(my_rpmsg, my_ept, 0x400, tx_buf, strlen(tx_buf));
        assert(result == 0);

        printf("%d\r\n", i);
    }
}

int main(void) {
    BOARD_RdcInit();
    
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    if (xTaskCreate(app_task, "APP_TASK", 512,  NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        PRINTF("\r\nFailed to create application task\r\n");
        for(;;);
    }

    vTaskStartScheduler();
    PRINTF("Failed to start FreeRTOS on core0.\n");
    for(;;);
}
