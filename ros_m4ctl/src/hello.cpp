#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rpmsg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include "ros/ros.h"
#include "ros_m4ctl/Start.h"
#include "ros_m4ctl/Stop.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello");
  ros::NodeHandle n;

  // boot firmware on m4core
  ros_m4ctl::Start srv;
  srv.request.firmware_path = "/ping.bin";
  if(!n.serviceClient<ros_m4ctl::Start>("/ros_m4ctl/start").call(srv)) {
    ROS_ERROR("failed to start core");
    return 1;
  }

  // TODO: wait for rpmsg channel announcement
  sleep(1);

  // load rpmsg_char kernel module for /dev/rpmsg_ctrl device
  system("modprobe rpmsg_char");
  
  // create endpoint for m4core rpmsg channel
  struct rpmsg_endpoint_info req;
  strcpy(req.name, "rpmsg-openamp-demo-channel");
  req.src = 0x11;
  req.dst = 0x1e;

  int fd = open("/dev/rpmsg_ctrl0", O_RDWR);
  ioctl(fd, RPMSG_CREATE_EPT_IOCTL, &req);

  // open created endpoint and start comunicating
  // TODO: open created channel!
  int d = open("/dev/rpmsg0", O_RDWR);
  const char hello_msg[] = "hello world!";
  write(d, hello_msg, sizeof(hello_msg));

  int ping = 0;
  while(ping < 100) {
    printf("Sending ping %d\n", ping);
    write(d, &ping, sizeof(ping));

    read(d, &ping, sizeof(ping));
    printf("Received ping %d\n", ping);
  }

  // stop core
  ros_m4ctl::Stop stopSrv;
  if(!n.serviceClient<ros_m4ctl::Stop>("/ros_m4ctl/stop").call(stopSrv)) {
    ROS_ERROR("failed to stop core");
    return 1;
  }

}
