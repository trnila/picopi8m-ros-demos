#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

void gpio_set_prop(int pin, const char *prop, const char* val) {
	char path[PATH_MAX];

	snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/%s", pin, prop);
	int fd = open(path, O_WRONLY);
	if(fd < 0) {
		perror("open");
		exit(1);
	}
	write(fd, val, strlen(val));
	close(fd);
}

void gpio_export(int pin) {
	char path[PATH_MAX];
	char buf[128];

	int fd = open("/sys/class/gpio/export", O_WRONLY);
	if(fd < 0) {
		perror("open");
		exit(1);
	}
	snprintf(buf, sizeof(buf), "%d", pin);
	if(write(fd, buf, strlen(buf)) != strlen(buf)) {
		// report error if already not exported
		if(errno != EBUSY) {
			perror("write");
			exit(1);
		}
	}
	close(fd);
}

int main() {
	int pin = 119;

	gpio_export(pin);
	gpio_set_prop(pin, "direction", "in");
	gpio_set_prop(pin, "edge", "falling");

	int fd = open("/sys/class/gpio/gpio119/value", O_RDONLY);
	if(!fd) {
		perror("open");
		return -1;
	}

	struct pollfd watch;
	watch.fd = fd;
	watch.events = POLLPRI | POLLERR;

	for(;;) {
		poll(&watch, 1, -1);

		char buf[128];
		read(fd, buf, sizeof(buf));

		printf("falling edge detected\n");

		// seek to the begin, because only there is new value
		lseek(fd, 0, SEEK_SET);
	}
	
}
