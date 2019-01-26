#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"

#ifndef RPMSG_LITE_SHMEM_BASE
#define RPMSG_LITE_SHMEM_BASE (0xB8000000U)
#endif

#ifndef RPMSG_LITE_NS_ANNOUNCE_STRING
#define RPMSG_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel-1"
#endif

#ifndef LOCAL_EPT_ADDR
#define LOCAL_EPT_ADDR (30)
#endif

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data)
{
}

class RpmsgHardware {
  public:
    RpmsgHardware():
      remote_addr(0),
      endpoint(NULL),
      rcv_queue(NULL),
      rpmsg(NULL),
      rx(NULL),
      rx_data_len(0),
      rx_data_pos(0) {}

    void init() {
      rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
      assert(rpmsg != NULL);
      rpmsg->link_state = 1;

      rcv_queue = rpmsg_queue_create(rpmsg);
      assert(rcv_queue != NULL);

      endpoint = rpmsg_lite_create_ept(rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, rcv_queue);
      assert(endpoint != NULL);

      rpmsg_ns_handle h = rpmsg_ns_bind(rpmsg, app_nameservice_isr_cb, NULL);
      assert(h != NULL);

      int result = rpmsg_ns_announce(rpmsg, endpoint, RPMSG_LITE_NS_ANNOUNCE_STRING, RL_NS_CREATE);
      assert(result == RL_SUCCESS);
    }

    int read() {
      if(rx_data_pos >= rx_data_len) {
        if(rx) {
          // release previous rx buffer
          int result = rpmsg_queue_nocopy_free(rpmsg, rx);
          assert(result == RL_SUCCESS);
        }

        rx = NULL;
        int result = rpmsg_queue_recv_nocopy(rpmsg, rcv_queue, &remote_addr, &rx, &rx_data_len, RL_DONT_BLOCK);
        assert(result == RL_SUCCESS || result == RL_ERR_NO_BUFF);

        if(result == RL_SUCCESS) {
          rx_data_pos = 0;
        } else {
          return -1;
        }
      }

      return rx[rx_data_pos++];
    };

    void write(uint8_t* data, int length) {
      if(remote_addr == 0) {
        // could not send message until we receive first message from linux
        return;
      }

      unsigned long tx_buf_len = -1;
      void *tx = rpmsg_lite_alloc_tx_buffer(rpmsg, &tx_buf_len, RL_BLOCK);
      assert(tx);
      assert(tx_buf_len >= (unsigned long) length);

      memcpy(tx, data, length);
      int result = rpmsg_lite_send_nocopy(rpmsg, endpoint, remote_addr, tx, length);
      assert(result == RL_SUCCESS);
    }

    unsigned long time() {
      return xTaskGetTickCount();
    }

  private:
    unsigned long remote_addr;
    struct rpmsg_lite_endpoint* endpoint;
    rpmsg_queue_handle rcv_queue;
    struct rpmsg_lite_instance *rpmsg;
    char *rx;
    int rx_data_len;
    int rx_data_pos;
};