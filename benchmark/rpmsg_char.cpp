#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <uapi/linux/rpmsg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include "benchmark.h"

int main(int argc, char **argv) {
  if(argc != 2) {
    fprintf(stderr, "Usage: %s total pings\n", argv[0]);
    return -1;
  }
  int total = atoi(argv[1]);

  struct rpmsg_endpoint_info req;
  strcpy(req.name, "rpmsg-openamp-demo-channel");
  req.src = 0x11;
  req.dst = 0x1e;

  int fd = open("/dev/rpmsg_ctrl0", O_RDWR);
  if(fd < 0) {
    perror("could not open /dev/rpmsg_ctrl: ");
    return -1;
  }

  int ret = ioctl(fd, RPMSG_CREATE_EPT_IOCTL, &req);
  close(fd);
  if(ret == -1) {
    return -1;
  }

  // open created endpoint and start comunicating
  // TODO: open created channel!
  int d = open("/dev/rpmsg0", O_RDWR);
  if(d < 0) {
    perror("open");
    return -1;
  }

  int ping = 0;
  while(ping < total) {
    benchmark_start();
    if(write(d, &ping, sizeof(ping)) <= 0) {
      close(d);
      break;
    }

    int got;
    if(read(d, &got, sizeof(got)) <= 0) {
      close(d);
      break;
    }

    printf("%llu\n", benchmark_stop());

    ping++;
    if(ping != got) {
      printf("error got %d, expected %d\n", got, ping);
      return 1;
    }
  }
}
