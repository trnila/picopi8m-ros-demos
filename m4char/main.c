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
      buf[len] = 0;
      printf("recv: '%s'\n", buf);
    }
  }

  int i = 0;
  for(;;) {
    int len = sprintf(buf, "ahoj %d", i);
    printf("send: '%s'\n", buf);
    write(fd, buf, len);
    usleep(100 * 1000);
    i++;
  }

  return 0;
}
