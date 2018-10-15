#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rpmsg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include "ros/ros.h"
#include "m4ctrl/Start.h"
#include "m4ctrl/Stop.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello");
  ros::NodeHandle n;

  // boot firmware on m4core
  m4ctrl::Start srv;
  srv.request.firmware_path = "/ping.bin";
  if(!n.serviceClient<m4ctrl::Start>("/m4ctrl/start").call(srv)) {
    ROS_ERROR("failed to start core");
    return 1;
  }

  // TODO: wait for rpmsg channel announcement
  sleep(1);
  
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
  m4ctrl::Stop stopSrv;
  if(!n.serviceClient<m4ctrl::Stop>("/m4ctrl/stop").call(stopSrv)) {
    ROS_ERROR("failed to stop core");
    return 1;
  }

}
