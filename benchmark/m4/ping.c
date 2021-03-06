/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "board.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_uart.h"
#include "rsc_table_rpmsg.h"

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMX8MQ_M4_USER_LINK_ID)

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

void app_task(void *param) {
    unsigned long remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;
    rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *my_rpmsg = create_rpmsg_from_resources();

    PRINTF("\r\nRPMSG benchmark...\r\n");

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_CHANNEL, RL_NS_CREATE);
    PRINTF("Nameservice announce sent.\r\n");

    uint32_t *rx_buf;
    uint32_t *tx_buf;
    int result;
    int len;
    int size;
    for(;;) {
	result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (unsigned long *) &remote_addr, (uint32_t **) &rx_buf, &len, RL_BLOCK);
	assert(result == 0);

	// dont send response to 'hello world' from imx_rpmsg_tty
	if(len == 4) {
	    tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
	    assert(tx_buf);

	    // ping ++
	    *tx_buf = *rx_buf + 1;

	    // send ping back
	    assert(rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len) == 0);
	}
	// return rx_buf
	assert(rpmsg_queue_nocopy_free(my_rpmsg, rx_buf) == 0);
    }

    for(;;);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    if(xTaskCreate(app_task, "APP_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
	PRINTF("\r\nFailed to create application task\r\n");
	for(;;);
    }

    vTaskStartScheduler();

    PRINTF("Failed to start FreeRTOS on core0.\n");
    for(;;);
}
