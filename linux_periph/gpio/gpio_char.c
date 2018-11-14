#include <linux/gpio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <limits.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>

int gpio_open(const char *name) {
	int fd;
	int i = 0;
	char path[PATH_MAX];

	for(;;) {
		snprintf(path, sizeof(path), "/dev/gpiochip%d", i);
		int fd = open(path, O_RDWR);
		if(fd < 0) {
			if(errno == ENOENT) {
				return -1;
			}
			return -2;
		}
		
		struct gpiochip_info info;
		ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info);

		if(strcmp(info.label, name) == 0) {
			return fd;
		}

		close(fd);
		i++;
	}
}

int main() {
	int fd = gpio_open("GPIO3");

	struct gpiohandle_request gpio_out, gpio_in;

	// request output pins
	gpio_out.flags = GPIOHANDLE_REQUEST_OUTPUT;
	gpio_out.lines = 2;
	gpio_out.lineoffsets[0] = 16; // GPIO3_IO16 
	gpio_out.lineoffsets[1] = 3;  // GPIO3_IO3
	gpio_out.default_values[0] = 0;
	gpio_out.default_values[1] = 0;
	strcpy(gpio_out.consumer_label, "demo-app-out");
	if(ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_out) != 0) {
		if(errno == EBUSY) {
			printf("Lines already used!\n");	
		} else {
			perror("ioctl");
		}
		return 1;
	}


	// request additional input pin
	gpio_in.flags = GPIOHANDLE_REQUEST_INPUT;
	gpio_in.lines = 1;
	gpio_in.lineoffsets[0] = 15; // GPIO3_15
	strcpy(gpio_in.consumer_label, "demo-app-in");
	if(ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_in) != 0) {
		if(errno == EBUSY) {
			printf("Lines already used!\n");	
		} else {
			perror("ioctl");
		}
		return 1;
	}
	
	// enable falling edge detection
	struct gpioevent_request gpio_falling;
	gpio_falling.lineoffset = 17; // GPIO3_17
	gpio_falling.handleflags = GPIOHANDLE_REQUEST_INPUT;
	gpio_falling.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;
	strcpy(gpio_falling.consumer_label, "demo-app-falling");
	ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &gpio_falling);


	struct gpiohandle_data data;
	data.values[0] = 0;
	data.values[1] = 1;


	struct timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 100 * 1000;
	for(;;) {
		fd_set fdset;
		FD_ZERO(&fdset);
		FD_SET(gpio_falling.fd, &fdset);


		int ret = select(gpio_falling.fd + 1, &fdset, NULL, NULL, &timeout);
	       
		if(ret < 0) {
			perror("select");
		} else if(ret == 0) {
			// flip outputs
			data.values[0] = !data.values[0];
			data.values[1] = !data.values[1];
			ioctl(gpio_out.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);

			// read pin value
			struct gpiohandle_data input_data;
			memset(&input_data, 0, sizeof(input_data));
			ioctl(gpio_in.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &input_data);
			printf("Input state: %d\n", input_data.values[0]);

			// reset timeout
			timeout.tv_usec = 100 * 1000;
		}
		
		if(FD_ISSET(gpio_falling.fd, &fdset)) {
			struct gpioevent_data event;
			read(gpio_falling.fd, &event, sizeof(event));
			printf("event %u detected at %llu ns\n", event.id, event.timestamp);
		}
	}
}
