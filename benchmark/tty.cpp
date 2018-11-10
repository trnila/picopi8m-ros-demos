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

  int speed = B115200;
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;   // 8 bits
  tty.c_cflag |= CREAD;                         // enable receiver

  // do not translate carriage return on binary data
  tty.c_iflag = 0;
  tty.c_lflag = 0;
  tty.c_oflag = 0;

  tty.c_cc[VMIN]  = sizeof(int);                // receive exatly 4 bytes
  tty.c_cc[VTIME] = 0;                          // wait forever for them

  tcsetattr (fd, TCSANOW, &tty);
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

  uint32_t ping = 0;
  while(ping < total) {
    benchmark_start();
    if(write(fd, &ping, sizeof(ping)) != sizeof(ping)) {
      perror("Failed to write");
      return -1;
    }
    ping++;

    uint32_t recv = 0;
    if(read(fd, &recv, sizeof(recv)) != sizeof(recv)) {
      perror("failed to read");
      return -1;
    }
    uint64_t time_ns = benchmark_stop();
    if(recv != ping) {
      fprintf(stderr, "ERR %d %d\n", recv, ping);
      return -1;
    }
    printf("%llu\n", time_ns);
  }
}
