#include <fcntl.h> 
#include <unistd.h>
#include <unistd.h>
#include <termios.h>
#include <memory>
#include <string.h>
#include <signal.h>
#include "benchmark.h"

int open_tty() {
  int fd = open("/dev/ttyRPMSG30", O_RDWR | O_NOCTTY | O_SYNC);
  if(fd < 0) {
    return fd;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    perror("tcgetattr");
    return -1;
  }

  cfsetspeed(&tty, B115200);
  cfmakeraw(&tty);

  tty.c_cc[VMIN]  = sizeof(int); // receive exatly 4 bytes
  tty.c_cc[VTIME] = 0;           // wait forever for them

  tcsetattr(fd, TCSANOW, &tty);
  return fd;
}

int main(int argc, char** argv) {
  if(argc != 2) {
    fprintf(stderr, "Usage: %s total pings\n", argv[0]);
    return -1;
  }

  int total = atoi(argv[1]);

  int fd = open_tty();
  if(fd < 0) {
    fprintf(stderr, "could not open /dev/ttyRPMSG30");
    return -1;
  }

  do_benchmark(fd, total);

  return 0;
}
