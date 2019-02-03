#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "ros.h"
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char buffer[32];

void app_task(void *param) {
    nh.initNode();
    nh.advertise(chatter);

    str_msg.data = buffer;

    int i = 0;
    for(;;) {
        snprintf(buffer, sizeof(buffer), "Hello world %d!", i++);
        chatter.publish(&str_msg);

        nh.spinOnce();
        vTaskDelay(1000);
    }
}

int main() {
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
    printf("Failed to start FreeRTOS on core\n");
    for(;;);
}
