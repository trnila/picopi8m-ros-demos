#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <ctype.h>

int main(int argc, char **argv) {
  if(argc < 2) {
    fprintf(stderr, "usage: %s /dev/ttyUSB0\n", argv[0]);
    exit(1);
  }

  int fd = open(argv[1], O_RDWR);
  if(fd < 0) {
    fprintf(stderr, "open(%s): %s\n", argv[1], strerror(errno));
    exit(1);
  }

  struct termios conf;
  // get current parameters
  if(tcgetattr(fd, &conf)) {
    perror("tcgetattr");
    exit(1);
  }
  // set raw mode at 115 200 bauds
  cfmakeraw(&conf);
  cfsetspeed(&conf, B115200);

  // apply configuration after flushing all data
  if(tcsetattr(fd, TCSAFLUSH, &conf) < 0) {
    perror("tcsetattr");
    exit(1);
  }

  for(;;) {
    char c;
    if(read(fd, &c, 1) != 1) {
      perror("read: ");
      exit(1);
    }

    c = toupper(c);
    write(fd, &c, 1);
  }

  return 0;
}
