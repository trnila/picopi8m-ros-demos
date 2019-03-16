#include <stdio.h>
#include "ros/ros.h"
#include "gpio_interrupts/Enable.h"
#include "gpio_interrupts/Counts.h"

using namespace gpio_interrupts;

void show_counts(const Counts &msg) {
  for(const Count &count: msg.counts) {
    printf("GPIO%d_IO%2d %10u\n", count.port, count.pin, count.count);
  }
}

void enable_interrupt(ros::NodeHandle &n, uint8_t port, uint8_t pin, uint8_t mode = EnableRequest::Enable) { 
  ros::ServiceClient svc = n.serviceClient<Enable>("/gpio_interrupts/enable");
  Enable e;;
  e.request.port = port;
  e.request.pin = pin;
  e.request.mode = mode;

  if(!svc.call(e) || !e.response.ok) {
    printf("failed to enable interrupt\n");
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pause");
  ros::NodeHandle n("pause");

  enable_interrupt(n, 4, 26);
  enable_interrupt(n, 3, 3);

  ros::Subscriber sub = n.subscribe("/gpio_interrupts/counts", 1000, show_counts);
  ros::spin();

  return 0;
}
