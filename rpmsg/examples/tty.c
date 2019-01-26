#include <termios.h>
#include <unistd.h>
#include <unistd.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>


int main() {
    for(;;) {
        int fd = open("/dev/ttyRPMSG30", O_RDWR);
        if(fd < 0) {
            perror("open");
            continue;
        }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            perror("tcgetattr");
            return -1;
        }

        cfmakeraw(&tty);
        cfsetspeed(&tty, B115200);
        tcsetattr (fd, TCSANOW, &tty);


        char buffer[128];
        int i = 0;
        for(;;) {
            snprintf(buffer, sizeof(buffer), "hello world %d", i++);
            if(write(fd, buffer, strlen(buffer)) < 0) {
               if(errno == EIO) {
                    break;
               }
            }
            usleep(1000 * 100);
        }
    }
}
