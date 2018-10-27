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
    printf("err\n");
  }

  int speed = B115200;
  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
	tty.c_iflag &= ~IGNBRK;
	tty.c_lflag = 0;
	tty.c_oflag = 0;
	tty.c_cc[VMIN]  = sizeof(int); // receive exatly 4 bytes
	tty.c_cc[VTIME] = 0;

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= 1; // parity
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tcsetattr (fd, TCSANOW, &tty);

  return fd;
}

int main() {
  int fd = open_tty();
  if(fd < 0) {
    fprintf(stderr, "could not open /dev/ttyRPMSG30");
    return -1;
  }

  uint32_t ping = 0;
	for(;;) {
    benchmark_start();
		if(write(fd, &ping, sizeof(ping)) != sizeof(ping)) {
      printf("Failed to write");
      return -1;
    }
    ping++;


    uint32_t recv = 0;
    if(read(fd, &recv, sizeof(recv)) != sizeof(recv)) {
      printf("failed to read");
      return -1;
    }
    uint64_t time_ns = benchmark_stop();
    if(recv != ping) {
      printf("ERR %d %d\n", recv, ping);
      sleep(1);
//      return -1;
    }
    printf("received: %d %d (%d)\n", ping, time_ns, recv);
	}

}
