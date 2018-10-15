#include "ros/ros.h"
#include "m4ctrl/Start.h"
#include "m4ctrl/Stop.h"
#include "m4core.h"

M4Core core;

bool start(m4ctrl::Start::Request &req, m4ctrl::Start::Response &res) {
  ROS_INFO("Boot firmware '%s' requested", req.firmware_path.c_str());
  core.boot_firmware(req.firmware_path.c_str());
  return true;
}

bool stop(m4ctrl::Stop::Request &req, m4ctrl::Stop::Response &res) {
  ROS_INFO("M4 stop requested");
  core.stop();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "m4ctrl");
  ros::NodeHandle n("m4ctrl");

  ros::ServiceServer startSrv = n.advertiseService("start", start);
  ros::ServiceServer stopSrv = n.advertiseService("stop", stop);

  ros::spin();
  return 1;
}
