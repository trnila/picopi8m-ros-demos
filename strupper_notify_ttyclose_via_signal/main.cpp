#include <fcntl.h> 
#include <unistd.h>
#include <unistd.h>
#include <termios.h>
#include <memory>
#include <string.h>
#include <signal.h>


volatile int m4_running = true;

void handler(int sig) {
  if(sig == SIGUSR1) {
    m4_running = false;
  } else if(sig == SIGUSR2) {
    m4_running = true;
  }
}


int open_tty() {
  int fd = -1;
  while((fd = open("/dev/ttyRPMSG30", O_RDWR | O_NOCTTY | O_SYNC)) < 0) {
    usleep(10 * 1000);
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
	tty.c_cc[VMIN]  = 0;
	tty.c_cc[VTIME] = 5;

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
  struct sigaction sa;
  sa.sa_handler = handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  sigaction(SIGUSR1, &sa, NULL);
  sigaction(SIGUSR2, &sa, NULL);


	char buffer[128];
  int running = false;
  int fd;
  int i = 0;
	for(;;) {
    if(running != m4_running) {
      if(m4_running) {
        fd = open_tty();
      } else {
        close(fd);
      }
      running = m4_running;
    }

    if(!running) {
      usleep(10 * 1000);
      continue;
    }

    int len = snprintf(buffer, sizeof(buffer), "hello world %d\n", i++);
		write(fd, buffer, len);
		len = read(fd, buffer, sizeof(buffer));
    write(1, buffer, len); 
	}

}
