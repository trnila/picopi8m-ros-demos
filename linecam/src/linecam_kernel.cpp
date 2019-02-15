#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "common.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "linecam_kernel");
  ros::NodeHandle n;
  ros::Publisher cam_publisher = n.advertise<std_msgs::UInt16MultiArray>("/linecam0", 1000);

  int fd = open("/dev/linecam0", O_RDONLY);
  if(fd < 0) {
    perror("open");
    return 1;
  }

  ros::Rate rate(100);

  std::vector<uint16_t> row(128);
  size_t bytes = row.size() * sizeof(row[0]);

  while(ros::ok()) {
    if(read(fd, row.data(), bytes) != bytes) {
      perror("read");
      return 1;
    }
    line_print(row.data(), row.size());
    
    std_msgs::UInt16MultiArray container;
    container.data = row;

    cam_publisher.publish(container);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
