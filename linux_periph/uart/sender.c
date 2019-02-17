#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

struct buffer {
  char buffer[256];
  int len;
};

int readline(int fd, char* dst, int size, struct buffer *buf) {
  int pos = 0;
   for(;;) {
      for(int i = 0; i < buf->len; i++) {
        dst[pos] = buf->buffer[i];
        if(dst[pos] == '\n') {
          // ignore \r 
          if(dst[pos - 1] == '\r') {
            pos--;
          }

          dst[pos] = 0;
          buf->len -= i + 1;
          memmove(buf->buffer, buf->buffer + i + 1, buf->len);
          return pos;
        }
        pos++;
      }

      int r = read(fd, buf->buffer, sizeof(buf->buffer));
      if(r <= 0) {
        return r;
      }

      buf->len = r;
   }
}

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


  struct buffer buf;
  buf.len = 0;
  char line[256];
  int i = 0;
  for(;;) {
    snprintf(line, sizeof(line), "Hello %d\r\n", i++);

    // send our line
    write(fd, line, strlen(line));

    // wait for response
    if(readline(fd, line, sizeof(line), &buf) <= 0) {
      perror("read");
      exit(1);
    }
    printf("recv: '%s'\n", line);
  }

  close(fd);
  return 0;
}
