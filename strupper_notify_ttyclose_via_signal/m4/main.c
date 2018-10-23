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

#define RPMSG_LITE_LINK_ID (RL_PLATFORM_IMX8MQ_M4_USER_LINK_ID)
#define RPMSG_LITE_SHMEM_BASE 0xB8000000
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-openamp-demo-channel"

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

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

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data) {}

void app_task(void *param) {
    volatile unsigned long remote_addr;
    struct rpmsg_lite_endpoint *volatile my_ept;
    volatile rpmsg_queue_handle my_queue;
    struct rpmsg_lite_instance *volatile my_rpmsg;
    volatile rpmsg_ns_handle ns_handle;

    PRINTF("\r\nRPMSG strupper FreeRTOS RTOS API Demo...\r\n");

    my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
    // TODO: we are immediatelly ready
    my_rpmsg->link_state = 1;
    PRINTF("Link is up!\r\n");

    my_queue = rpmsg_queue_create(my_rpmsg);
    my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
    ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
    rpmsg_ns_announce(my_rpmsg, my_ept, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);
    PRINTF("Nameservice announce sent.\r\n");

    char *rx_buf;
    char *tx_buf;
    int result;
    int len;
    int size;
    for(;;) {
        result = rpmsg_queue_recv_nocopy(my_rpmsg, my_queue, (unsigned long *) &remote_addr, (char **) &rx_buf, &len, RL_BLOCK);
        assert(result == 0);

        rx_buf[len] = 0;
        printf("Got: '%s'\r\n", rx_buf);

        tx_buf = rpmsg_lite_alloc_tx_buffer(my_rpmsg, &size, RL_BLOCK);
        assert(tx_buf);

        for(int i = 0; i < len; i++) {
          tx_buf[i] = rx_buf[i];
          if(tx_buf[i] >= 'a' && tx_buf[i] <= 'z') {
            tx_buf[i] -= 'a' - 'A';
          }
        }

        // return rx_buf
        assert(rpmsg_queue_nocopy_free(my_rpmsg, rx_buf) == 0);
        assert(rpmsg_lite_send_nocopy(my_rpmsg, my_ept, remote_addr, tx_buf, len) == 0);
    }

    for(;;);
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
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
