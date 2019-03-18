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

int main() {
	int fd = open("/dev/gpio4", O_RDWR);
	if(fd < 0) {
		perror("open");
		return 1;
	}

	struct gpiohandle_request gpio_out;
	gpio_out.flags = GPIOHANDLE_REQUEST_OUTPUT;
	gpio_out.lines = 1;
	gpio_out.lineoffsets[0] = 26; // GPIO4_IO26 
	gpio_out.default_values[0] = 0;
	strcpy(gpio_out.consumer_label, "gpio-concurency-test");
	if(ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_out) != 0) {
		if(errno == EBUSY) {
			printf("Lines already used!\n");	
		} else {
			perror("ioctl");
		}
		return 1;
	}

	struct gpiohandle_data set, get;
	set.values[0] = 0;

	for(;;) {
		if(ioctl(gpio_out.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &set) != 0) {
			perror("ioctl(set lines)");
			return 1;
		}
		if(ioctl(gpio_out.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &get) != 0) {
			perror("ioctl(get lines)");
			return 1;
		}


		if(set.values[0] != get.values[0]) {
			printf("GPIO lost pin value because of concurency\n");
			return 1;
		}
		printf("OK set and read same value %d\n", set.values[0]);

		set.values[0] = !set.values[0];
	}
}
