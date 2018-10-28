#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>

char buf[128];

int main() {
  int fd = open("/dev/m4char", O_RDWR);
  if(fork() == 0) {
    for(;;) {
      int len = read(fd, buf, sizeof(buf));
      if(len <= 0) {
        printf("read = %d\n", len);
      } else {
        buf[len] = 0;
        printf("recv: '%s'\n", buf);
      }
    }
  }

  int i = 0;
  for(;;) {
    int len = sprintf(buf, "ahoj %d", i);
    printf("send: '%s'\n", buf);
    int r = write(fd, buf, len);
    if(r != len) {
      printf("write wrote %d but expected %d\n", r, len);
    }
    usleep(1* 1000);
    i++;
  }

  return 0;
}
