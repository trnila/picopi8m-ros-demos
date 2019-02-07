#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "ros/ros.h"
#include "ros_m4ctl/Start.h"
#include "ros_m4ctl/Stop.h"

template<typename...Args>
bool m4ctl(Args... args) {
  pid_t child = fork();
  if(child < 0) {
    perror("fork:");
    return false;
  }

  if(child == 0) {
    execlp("m4ctl", "m4ctl", args..., NULL);
    exit(1);
  }
  
  int status;
  waitpid(child, &status, 0);

  return WEXITSTATUS(status) == 0;
}


bool start(ros_m4ctl::Start::Request &req, ros_m4ctl::Start::Response &res) {
  ROS_INFO("Boot firmware '%s' requested", req.firmware_path.c_str());
  m4ctl("start", req.firmware_path.c_str());
  return true;
}

bool stop(ros_m4ctl::Stop::Request &req, ros_m4ctl::Stop::Response &res) {
  ROS_INFO("M4 stop requested");
  m4ctl("stop");
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ros_m4ctl");
  ros::NodeHandle n("ros_m4ctl");

  ros::ServiceServer startSrv = n.advertiseService("start", start);
  ros::ServiceServer stopSrv = n.advertiseService("stop", stop);

  ros::spin();
  return 1;
}
