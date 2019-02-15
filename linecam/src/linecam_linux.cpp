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
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "common.h"

void set_pins(int fd, bool cam_si, bool cam_clk) {
  struct gpiohandle_data data{};
  data.values[0] = cam_si;
  data.values[1] = cam_clk;
  if(ioctl(fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) != 0) {
    perror("set_pins ioctl");
    exit(1);
  }
}

uint16_t adc_read(int spi_fd) {
  uint32_t rx;
  uint32_t tx = 0; // channel 0

  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long) &tx;
  tr.rx_buf = (unsigned long) &rx;
  tr.len = 2;
  tr.delay_usecs = 0;
  tr.speed_hz = 12000000;
  tr.bits_per_word = 16;

  if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) != 2) {
    perror("spi transfer ioctl: ");
    exit(1);
  }

  return rx;
}

void measure(int gpio_fd, int spi_fd, uint16_t *frame, int len) { 
  set_pins(gpio_fd, 0, 0);
  set_pins(gpio_fd, 1, 0); // SI goes up
  set_pins(gpio_fd, 1, 1); // CLK goes up
  set_pins(gpio_fd, 0, 1);


  for(int i = 0; i < len; i++) {
      set_pins(gpio_fd, 0, 0);
      frame[i] = adc_read(spi_fd);
      set_pins(gpio_fd, 0, 1);
  }
}

int gpio_init() {
  int fd = open("/dev/gpio4", O_RDWR);
  if(fd < 0) {
    perror("open /dev/gpio4");
    exit(1);
  }

  struct gpiohandle_request gpio_out{};
  gpio_out.flags = GPIOHANDLE_REQUEST_OUTPUT;
  gpio_out.lines = 2;
  gpio_out.lineoffsets[0] = 23; // GPIO4_23
  gpio_out.lineoffsets[1] = 26; // GPIO4_26
  gpio_out.default_values[0] = 0;
  gpio_out.default_values[1] = 0;
  strcpy(gpio_out.consumer_label, "linecam");
  if(ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &gpio_out) != 0) {
    if(errno == EBUSY) {
      printf("Lines already used!\n");	
    } else {
      perror("ioctl");
    }
    exit(1);
  }

  // we have acquired descriptor for two output pins, so we dont need descriptor for that bank anymore
  close(fd);
  return gpio_out.fd;
}

int spi_init() {
  int spi_fd = open("/dev/spidev1.0", O_RDWR);
  if(spi_fd < 0) {
    perror("open spi");
    exit(1);
  }

  // set CPOL = 1
  uint8_t mode = SPI_CPOL;
  if(ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) != 0) {
    perror("ioctl CPOL=1: ");
    exit(1);
  }

  uint8_t bits_per_word = 16;
  if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) != 0) {
    perror("ioctl bits_per_word: ");
    exit(1);
  }

  return spi_fd;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "linecam_linux");
  ros::NodeHandle n;
  ros::Publisher cam_publisher = n.advertise<std_msgs::UInt16MultiArray>("/linecam0", 1000);

  int gpio_fd = gpio_init();
  int spi_fd = spi_init();

  std::vector<uint16_t> row(128);
  while(ros::ok()) {
    measure(gpio_fd, spi_fd, row.data(), row.size());
    line_print(row.data(), row.size());
    
    std_msgs::UInt16MultiArray container;
    container.data = row;

    cam_publisher.publish(container);
    ros::spinOnce();
  }

  return 0;
}
