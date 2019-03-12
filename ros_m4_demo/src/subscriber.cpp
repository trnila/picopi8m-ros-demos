#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

void hello_cb(const std_msgs::String &msg) {
  printf("%s\n", msg.data.c_str());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle n("fake_sensor_hello");

  ros::Subscriber sub = n.subscribe("/ros_m4_demo/hello", 1000, hello_cb);
  ros::spin();

  return 0;
}
