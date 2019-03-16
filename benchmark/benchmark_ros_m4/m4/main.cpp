#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ros.h"
#include "std_msgs/UInt32.h"

ros::NodeHandle nh;

std_msgs::UInt32 ping_msg;
ros::Publisher topic_pub("/benchmark/m4", &ping_msg);

ros::Subscriber<std_msgs::UInt32> topic_sub("/benchmark/linux", [](const std_msgs::UInt32& msg) {
    ping_msg.data = msg.data + 1;
    topic_pub.publish(&ping_msg);
});

void rosserial_task(void *param) {
    for(;;) {
      nh.spinOnce();
    }
}

int main(void) {
  BOARD_RdcInit();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("started\r\n");

  nh.initNode();
  nh.subscribe(topic_sub);
  nh.advertise(topic_pub);

  if(xTaskCreate(rosserial_task, "ROS_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINTF("\r\nFailed to create task\r\n");
    for(;;);
  }

  vTaskStartScheduler();

  PRINTF("Failed to start FreeRTOS on core0.\n");
  for(;;);
}
