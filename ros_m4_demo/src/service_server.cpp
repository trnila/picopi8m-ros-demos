#include <stdio.h>
#include "ros/ros.h"
#include "ros_m4_demo/Addition.h"

bool handle_addition_service(ros_m4_demo::AdditionRequest &req, ros_m4_demo::AdditionResponse &res) {
  printf("Calculating %d + %d\n", req.a, req.b);
  res.result = req.a + req.b;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pause");
  ros::NodeHandle n("pause");

  ros::ServiceServer service = n.advertiseService("/ros_m4_demo/addition", handle_addition_service);
  ros::spin();

  return 0;
}
