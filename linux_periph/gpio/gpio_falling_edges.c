#include <linux/gpio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/poll.h>

int main() {
	int fd = open("/dev/gpiochip3", O_RDWR);

	struct gpioevent_request gpio_falling;
	gpio_falling.lineoffset = 26; // GPIO4_26
	gpio_falling.handleflags = GPIOHANDLE_REQUEST_INPUT;
	gpio_falling.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;
	strcpy(gpio_falling.consumer_label, "demo-app-falling");
	ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &gpio_falling);

	struct pollfd watch;
	watch.fd = fd;
	watch.revents = POLLIN;

	while(poll(&watch, 1, -1) > 0) {
			struct gpioevent_data event;
			read(gpio_falling.fd, &event, sizeof(event));
			printf("event %u detected at %llu ns\n", event.id, event.timestamp);
	}
}
