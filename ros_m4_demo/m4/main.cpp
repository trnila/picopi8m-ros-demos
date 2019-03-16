#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "board.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "ros_m4_demo/Location.h"
#include "ros_m4_demo/Pause.h"
#include "ros_m4_demo/Print.h"
#include "ros_m4_demo/SetSigma.h"
#include "ros_m4_demo/Addition.h"

volatile float sigma = 0.2;

ros::NodeHandle nh;
TaskHandle_t measure_task;

// Topic publisher, sends data TO Linux side
ros_m4_demo::Location location;
ros::Publisher measurements_topic("/ros_m4_demo/location", &location);

// Topic subscriber, receives data FROM Linux side
// $ rostopic pub /ros_m4_demo/print ros_m4_demo/Print "Hello World" 64
// src/publisher.cpp on Linux side
ros::Subscriber<ros_m4_demo::Print> print_sub("/ros_m4_demo/print", [](const ros_m4_demo::Print& msg) {
  printf("Print: '%s', %d\r\n", msg.text, msg.num); 
});

// Service executed on M4 core
// $ rosservice call ros_m4_demo/pause false
// src/service_call.cpp on Linux side
int pause_count, resume_count;
void pause_cb(const ros_m4_demo::PauseRequest &req, ros_m4_demo::PauseResponse &resp) {
      if(req.pause) {
        printf("Pausing measurements\r\n");
        vTaskSuspend(measure_task);
        resp.count = ++pause_count;
      } else {
        printf("Resuming measurements\r\n");
        vTaskResume(measure_task);
        resp.count = ++resume_count;
      }
}
ros::ServiceServer<ros_m4_demo::PauseRequest, ros_m4_demo::PauseResponse> pause_service("/ros_m4_demo/pause", pause_cb);

// $ rosservice call ros_m4_demo/set_sigma 7.5
ros::ServiceServer<ros_m4_demo::SetSigmaRequest, ros_m4_demo::SetSigmaResponse> set_sigma_service("/ros_m4_demo/set_sigma", [](const ros_m4_demo::SetSigmaRequest& req, ros_m4_demo::SetSigmaResponse &empty) {
    sigma = req.sigma;
    nh.loginfo("Set new sigma: %f", sigma);
});


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

// handle incomming ROS events (subscribed topics, service calls)
void rosserial_task(void *param) {
    for(;;) {
        // our hardware read() implementation blocks until timeout
        // so here is no busy waiting
        // when timeout occurs, it sends to Linux keep-alive message
        nh.spinOnce();
    }
}

// Topic publisher, sends data TO Linux side
// $ rostopic echo /ros_m4_demo/hello
// src/subscriber.cpp on Linux side
void location_task(void *param) {
  uint32_t t = 0;
  float prev_pos = 0;
  float prev_velocity = 0;
  for(;;) {
    location.real_position = sin(t/10.0);

    // add some gauss noise to faked measured position
    float observed_pos = location.real_position + gaussrand() * sigma;

    location.position = observed_pos;
    location.velocity = observed_pos - prev_pos; 
    location.acceleration = location.velocity - prev_velocity;

    prev_pos = location.position;
    prev_velocity = location.velocity;
    t++;

    measurements_topic.publish(&location);

    printf("sent %d\r\n", t);
    vTaskDelay(100);
  }
}

// Topic publisher, sends data TO Linux side
// $ rostopic echo /ros_m4_demo/hello
// src/subscriber.cpp on Linux side
std_msgs::String hello_msg;
ros::Publisher hello_topic("/ros_m4_demo/hello", &hello_msg);
void hello_task(void*) {
  char buf[32];
  int t = 0;
  hello_msg.data = buf;
  for(;;) {
    snprintf(buf, sizeof(buf), "Hello %d!", t++);
    hello_topic.publish(&hello_msg);
    vTaskDelay(200);
  }
}

// Call service on Linux side
// src/service_server.cpp 
auto addition_service_client = ros::ServiceClient<ros_m4_demo::AdditionRequest, ros_m4_demo::AdditionResponse>("/ros_m4_demo/addition");
void calculate_task(void*) {
  ros_m4_demo::AdditionRequest req;
  ros_m4_demo::AdditionResponse res;

  req.a = 0;
  req.b = 100;
  for(;;) {
    req.a++;
    req.b += 10;

    addition_service_client.call(req, res);
    printf("Addition(%d, %d) = %d\r\n", req.a, req.b, res.result);

    vTaskDelay(1000);
  }

}

int main(void) {
  BOARD_RdcInit();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("started\r\n");

  nh.initNode();
  nh.subscribe(print_sub);
  nh.advertise(measurements_topic);
  nh.advertise(hello_topic);
  nh.advertiseService(pause_service);
  nh.advertiseService(set_sigma_service);
  nh.serviceClient(addition_service_client);

  if(xTaskCreate(calculate_task, "CALC_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINTF("\r\nFailed to create task\r\n");
    for(;;);
  }

  if(xTaskCreate(rosserial_task, "ROS_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINTF("\r\nFailed to create task\r\n");
    for(;;);
  }

  if(xTaskCreate(hello_task, "HELLO_TASK", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    PRINTF("\r\nFailed to create task\r\n");
    for(;;);
  }

  if(xTaskCreate(location_task, "LOC_TASK", 256, NULL, tskIDLE_PRIORITY + 1, &measure_task) != pdPASS) {
    PRINTF("\r\nFailed to create location task\r\n");
    for(;;);
  }

  vTaskStartScheduler();

  PRINTF("Failed to start FreeRTOS on core0.\n");
  for(;;);
}
