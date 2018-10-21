#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/rpmsg.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {
  // load rpmsg_char kernel module for /dev/rpmsg_ctrl device
  system("modprobe rpmsg_char");
  
  struct rpmsg_endpoint_info req;
  strcpy(req.name, "rpmsg-openamp-demo-channel");
  req.src = 0x11;
  req.dst = 0x1e;

  int ping = 0;
  for(;;) {
    // TODO: required for now, otherwise kernel crashes sometimes
    sleep(1);

    // create our endpoint for m4core rpmsg channel
    int fd = open("/dev/rpmsg_ctrl0", O_RDWR);
    if(fd < 0) {
      perror("could not open /dev/rpmsg_ctrl\n");
      continue;
    }
    ioctl(fd, RPMSG_CREATE_EPT_IOCTL, &req);
    close(fd);

    // open created endpoint and start comunicating
    // TODO: open created channel!
    int d = open("/dev/rpmsg0", O_RDWR);
    if(d < 0) {
      perror("open");
      continue;
    }

    for(;;) {
      printf("Sending ping %d\n", ping);
      if(write(d, &ping, sizeof(ping)) <= 0) {
        close(d);
        break;
      }

      int got;
      if(read(d, &got, sizeof(got)) <= 0) {
        close(d);
        break;
      }

      ping++;
      if(ping != got) {
        printf("error got %d, expected %d\n", got, ping);
        return 1;
      }
      
      printf("Received ping %d\n", ping);
    }
  }
}
