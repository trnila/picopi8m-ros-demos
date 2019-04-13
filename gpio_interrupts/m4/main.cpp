#include <stdio.h>
#include <vector>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ros.h"
#include "gpio_interrupts/Counts.h"
#include "gpio_interrupts/Enable.h"
#include "interrupts.h"

using namespace gpio_interrupts;

ros::NodeHandle nh;

gpio_interrupts::Count states[MAX_PINS];
gpio_interrupts::Counts counts_msg;
ros::Publisher counts_topic("/gpio_interrupts/counts", &counts_msg);

IRQMode map_mode(uint8_t mode) {
  switch(mode) {
    case EnableRequest::Enable:
      return IRQMode::Both;
    case EnableRequest::Rising:
      return IRQMode::Rising;
    case EnableRequest::Falling:
      return IRQMode::Falling;
  }

  nh.logerror("unknown irq mode %d", mode);
  return IRQMode::Both;
}

// $ rosservice call gpio_interrupts/enable 4 26 1 
ros::ServiceServer<EnableRequest, EnableResponse> enable_service("/gpio_interrupts/enable", [](const EnableRequest& req, EnableResponse &res) {
    nh.loginfo("enable irq port=%d pin=%d mode=%d", req.port, req.pin, req.mode);
    if(req.mode == EnableRequest::Disable) {
      disable(req.port, req.pin);
    } else {
      enable(req.port, req.pin, map_mode(req.mode));
    }
    res.ok = true;
});

// $ rostopic echo /gpio_interrupts/counts
void publish_task(void *param) {
  nh.advertise(counts_topic);
  counts_msg.counts = states;

  for(;;) {
    int i = 0;
    for(int pin = 0; pin < MAX_PINS; pin++) {
      if(enabled[pin]) {
        states[i].port = (pin / 32) + 1;
        states[i].pin = pin % 32;
        states[i].count = counts[pin];
        i++;
      }
    }

    counts_msg.counts_length = i;
    counts_topic.publish(&counts_msg);

    vTaskDelay(100);
  }
}

void ros_task(void *param) {
    nh.advertiseService(enable_service);

    for(;;) {
        nh.spinOnce();
    }
}

int main(void) {
    BOARD_RdcInit();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    interrupts_init();

    nh.initNode();

    //enable(4, 26, IRQMode::Both);

    if (xTaskCreate(publish_task, "pub_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create ros task\r\n"); 
        for(;;);
    }

    if (xTaskCreate(ros_task, "ROS_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        printf("\r\nFailed to create ros task\r\n"); 
        for(;;);
    }

    vTaskStartScheduler();
    printf("Failed to start FreeRTOS\n");
    for(;;);
}
