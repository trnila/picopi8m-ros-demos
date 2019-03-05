#include <termios.h>
#include <unistd.h>
#include <unistd.h>
#include <fcntl.h> 
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

int main() {
    int fd = open("/dev/m4char", O_RDWR);
    if(fd < 0) {
        perror("open");
        exit(1);
    }
    char buffer[128];

    if(fork() == 0) {
        for(;;) {
            int n = read(fd, buffer, sizeof(buffer));
            if(n > 0) {
                buffer[n] = 0;
                printf("received: '%s'\n", buffer);
            } else if(errno == EPIPE) {
                usleep(1000 * 100);
            }
        }
    }

    int i = 0;
    for(;;) {
        snprintf(buffer, sizeof(buffer), "hello world %d", i++);
        if(write(fd, buffer, strlen(buffer)) < 0) {
           if(errno == EPIPE || errno == ENOMEM) {
                usleep(1000 * 100);
           }
        }
    }
}
