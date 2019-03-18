#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_m4_demo/Location.h"

using namespace ros_m4_demo;

void hello_cb(const std_msgs::String &msg) {
  printf("%s\n", msg.data.c_str());
}

void location_cb(const Location &msg) {
  printf("%f %f %f\n", msg.position, msg.velocity, msg.acceleration);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n("subscriber");

  ros::Subscriber sub = n.subscribe("/ros_m4_demo/hello", 1000, hello_cb);
  ros::Subscriber sub2 = n.subscribe("/ros_m4_demo/location", 1000, location_cb);
  ros::spin();

  return 0;
}
