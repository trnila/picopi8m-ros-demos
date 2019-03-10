#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rpmsg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>
#include <errno.h>
#include "benchmark.h"

int main(int argc, char **argv) {
  if(argc != 2) {
    fprintf(stderr, "Usage: %s total pings\n", argv[0]);
    return -1;
  }
  int total = atoi(argv[1]);

  int fd = open("/dev/m4char", O_RDWR);
  if(fd < 0) {
    perror("could not open /dev/m4char: ");
    return -1;
  }

  int ping = 0;
  while(write(fd, &ping, sizeof(ping)) <= 0 && errno == EPIPE) {
    usleep(1000);
  }

  int got;
  if(read(fd, &got, sizeof(got)) != sizeof(got) || got != 1) {
    fprintf(stderr, "invalid read: %d\n", got);
    return -1;
  }

  ping = 0;
  while(ping < total) {
    benchmark_start();
    if(write(fd, &ping, sizeof(ping)) <= 0) {
      perror("write");
      return -1;
    }

    if(read(fd, &got, sizeof(got)) <= 0) {
      perror("read");
      return -1;
    }

    printf("%llu\n", benchmark_stop());

    ping++;
    if(ping != got) {
      printf("error got %d, expected %d\n", got, ping);
      return 1;
    }
  }
}
