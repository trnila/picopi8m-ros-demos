#include <stdio.h>
#include "board.h"
#include "FreeRTOS.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"
#include "task.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_rdc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

void HardFault_Handler(void) {
  printf("hard fault\r\n");
  for(;;);
}


void MemManage_Handler(void) {
  // [0] - fetch from non-executable location
  // [1] - load/store from non-accessible location
  uint32_t cfsr  = SCB->CFSR;

  uint32_t mmfar = SCB->MMFAR; // address of location that generated memmanage fault
  uint32_t bfar  = SCB->BFAR;  // address of location that generated busfault

  printf("MemManage fault\r\n");
  printf("cfsr = %x\r\n", cfsr);
  printf("mmfar = %x\r\n", mmfar);
  printf("bfar = %x\r\n", bfar);
  for(;;);
}

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data) {}

char rx_buf[512];
char tx_buf[512];
void app_task(void *param) {
  volatile unsigned long remote_addr;
  struct rpmsg_lite_endpoint *volatile my_ept;
  volatile rpmsg_queue_handle my_queue;
  struct rpmsg_lite_instance *volatile my_rpmsg;
  volatile rpmsg_ns_handle ns_handle;

  PRINTF("\r\naRPMSG strupper FreeRTOS RTOS API Demo...\r\n");

  my_rpmsg = rpmsg_lite_remote_init((void *)RPMSG_LITE_SHMEM_BASE, RPMSG_LITE_LINK_ID, RL_NO_FLAGS);
  my_rpmsg->link_state = 1;

  my_queue = rpmsg_queue_create(my_rpmsg);
  my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR, rpmsg_queue_rx_cb, my_queue);
  ns_handle = rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
  rpmsg_ns_announce(my_rpmsg, my_ept, "m4-channel", RL_NS_CREATE);
  PRINTF("Nameservice announce sent.\r\n");

  // this will cause MPU fault
  //*(volatile uint32_t*) (0x40000000);

  for(;;) {
    int len = 0;
    assert(rpmsg_queue_recv(my_rpmsg, my_queue, (unsigned long *)&remote_addr, rx_buf, sizeof(rx_buf), &len, RL_BLOCK) == 0);
    rx_buf[len] = 0;
    printf("recv: '%s' (%d)\r\n", rx_buf, len);

    for(int i = 0; i < len; i++) {
      tx_buf[i] = rx_buf[i];
      if(tx_buf[i] >= 'a' && tx_buf[i] <= 'z') {
        tx_buf[i] -= 'a' - 'A';
      }
    }

    rpmsg_lite_send(my_rpmsg, my_ept, remote_addr, tx_buf, len, RL_BLOCK);
  }

  for(;;);
}

volatile int x;

int main(void) {
  BOARD_RdcInit();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  __DMB();

  /* Disable the MPU until we fully configure it */
  MPU->CTRL = 0;

  /* configure full access/0x3 (AP) from 0x2000_0000 to 0x4000_0000 (~536 Mbytes - 2^29) */
  MPU->RBAR = (0x20000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (29 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* configure full aceess to code - 256 kbytes - 2^8 */
  MPU->RBAR = (0x1FFF0000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (1 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (8 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* configure full access to rpmsg virtqueue
   * 0xb800 0000 - 0xb801 0000 (64 Kbytes - 2^17)
   */
  MPU->RBAR = (0xB8000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (2 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (17 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;


  /* configure full access to rpmsg descriptors
   * TODO: starting address yet unknown
   * size: 2 * 256 * 512
   *  vrings * buffers_count * size_of_buffer
   */
  MPU->RBAR = (0xB4340000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (3 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (18 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* enable MemManage handlers */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

  /* enable mpu */
  MPU->CTRL = MPU_CTRL_ENABLE_Msk;

  /* Memory barriers to ensure subsequence data & instruction
   * transfers using updated MPU settings.
   */
  __DSB();
  __ISB();

  if(xTaskCreate(app_task, "APP_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINTF("\r\nFailed to create application task\r\n");
    for(;;);
  }

  vTaskStartScheduler();
  printf("freertos failed");
  for(;;);
}
