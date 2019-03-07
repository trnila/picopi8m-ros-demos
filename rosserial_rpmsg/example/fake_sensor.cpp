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
#include "demo_fake_m4_sensor/Location.h"
#include "demo_fake_m4_sensor/Pause.h"

volatile float sigma = 1.0;
volatile bool paused = 0;

ros::NodeHandle nh;

struct demo_fake_m4_sensor::Location loc{};
ros::Publisher measurements("/fake_sensor/location", &loc);

void service_control(const demo_fake_m4_sensor::Pause::Request &req, demo_fake_m4_sensor::Pause::Response &res);
ros::ServiceServer<demo_fake_m4_sensor::Pause::Request, demo_fake_m4_sensor::Pause::Response> control_srv("/fake_sensor/control", service_control);


void service_control(const demo_fake_m4_sensor::Pause::Request &req, demo_fake_m4_sensor::Pause::Response &res) {
  paused = req.pause;
  printf("paused: %d\r\n", req.pause);
  nh.loginfo(paused ? "paused" : "resumed");
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

void app_task(void *param) {
  nh.initNode();
  nh.advertise(measurements);
  nh.advertiseService(control_srv);

  int t = 0;
  float prev_pos = 0;
  float prev_velocity = 0;
  for(;;) {
    loc.real_position = sin(t / 10.0);
    loc.position = loc.real_position + gaussrand() * sigma;
    loc.velocity = loc.position - prev_pos; 
    loc.acceleration = loc.velocity - prev_velocity;

    prev_pos = loc.position;
    prev_velocity = loc.velocity;

    if(!paused) {
      measurements.publish(&loc);
    }

    nh.spinOnce();

    float new_sigma;
    if(nh.getParam("/fake_sensor/sigma", &new_sigma) && sigma != new_sigma) {
      printf("Sigma changed from %f to %f\r\n", sigma, new_sigma);
      sigma = new_sigma;
    }

    vTaskDelay(1);
    t++;
  }
}

int main() {
  BOARD_RdcInit();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("rosserial_rpmsg started\r\n");

  if (xTaskCreate(app_task, "APP_TASK", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    printf("\r\nFailed to create application task\r\n"); 
    for(;;);
  }

  vTaskStartScheduler();
  printf("Failed to start FreeRTOS on core\n");
  for(;;);
}
