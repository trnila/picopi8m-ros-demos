#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "demo_fake_m4_sensor/Location.h"
#include "demo_fake_m4_sensor/Pause.h"
#include "../m4/data.h"

int fd;

bool service_control(demo_fake_m4_sensor::Pause::Request &req, demo_fake_m4_sensor::Pause::Response &res) {
  Command cmd{};
  cmd.type = CMD_PAUSE;
  cmd.u8 = req.pause;

  if(write(fd, &cmd, sizeof(cmd)) != sizeof(cmd)) {
    ROS_WARN("Failed to send command pause");
    return false;
  }
  ROS_INFO("pause: %d", req.pause);

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bridge");
  ros::NodeHandle n("fake_sensor");

  // open char device for communication with m4 core
  fd = open("/dev/m4char", O_RDWR);
  if(fd < 0) {
    ROS_ERROR("Could not open /dev/m4char, have you loaded module rpmsg_m4char.ko?");
    return -1;
  }

  // handle callbacks (for example service_control) in thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // register service for pausing m4 core
  // from shell, you can issue pause with:
  // $ rosservice call fake_sensor/pause true 
  ros::ServiceServer service = n.advertiseService("pause", service_control);

  // publish all received measurements in /<node name>/location
  ros::Publisher location_pub = n.advertise<demo_fake_m4_sensor::Location>("location", 1000);
  float prev_sigma = -1;
  while (ros::ok()) {
    Measurement m{};
    int len = read(fd, &m, sizeof(m));
    if(len <= 0) {
      ROS_WARN("invalid read, remote side is down?");
      prev_sigma = -1;
      usleep(100 * 1000);
      continue;
    }

    demo_fake_m4_sensor::Location ros_msg;
    ros_msg.position = m.position;
    ros_msg.velocity = m.velocity;
    ros_msg.acceleration = m.acceleration;
    ros_msg.real_position = m.real_position;
    location_pub.publish(ros_msg);

    float sigma;
    n.getParamCached("/sigma", sigma);
    sigma = fabs(sigma);
    if(fabs(prev_sigma - sigma) > 0.00001) {

      Command cmd{};
      cmd.type = CMD_SIGMA;
      cmd.f32 = sigma;

      if(write(fd, &cmd, sizeof(cmd)) != sizeof(cmd)) {
        ROS_WARN("Failed to send sigma");
      } else {
        ROS_INFO("sent new sigma: %f", sigma);
      }
      prev_sigma = sigma;
    }
  }

  return 0;
}
