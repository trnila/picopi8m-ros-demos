#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <errno.h>
#include <limits.h>

#define DS1621_READ_TEMP      0xAA
#define DS1621_READ_COUNTER   0xA8
#define DS1621_READ_SLOPE     0xA9
#define DS1621_START_CONVERT  0xEE
#define DS1621_ADDR           0x48


/*
 * read from slave in single I2C transaction with repeated start
 */
uint8_t i2c_read_byte(int fd, uint8_t addr, uint8_t reg) {
  // we will make two transfers in single transaction
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data xfer;
  xfer.msgs = msgs;
  xfer.nmsgs = 2;

  // in first transaction we will send register address
  msgs[0].addr = addr;
  msgs[0].flags = 0;
  msgs[0].buf = &reg;
  msgs[0].len = sizeof(reg);

  // in second transaction we will read byte from slave
  uint8_t rx;
  msgs[1].addr = addr;
  msgs[1].flags = I2C_M_RD;
  msgs[1].buf = &rx;
  msgs[1].len = sizeof(rx);

  // make transfer
  if(ioctl(fd, I2C_RDWR, &xfer) < 0) {
    perror("ioctl");
  }

  return rx;
}

int main(int argc, char **argv) {
  if(argc <= 1) {
    fprintf(stderr, "Usage: %s (i2c id)\n", argv[0]);
    return 1;
  }

  char path[PATH_MAX];
  snprintf(path, sizeof(path), "/dev/i2c-%s", argv[1]);

  int fd = open(path, O_RDWR);
  if(fd < 0) {
    printf("open(%s): %s\n", path, strerror(errno));
    exit(1);
  }

  // set slave address when using simplex
  if(ioctl(fd, I2C_SLAVE, DS1621_ADDR) < 0) {
    perror("ioctl");
    exit(1);
  }

  // start measuring temperature by sending START_CONVERT
  uint8_t byte;
  byte = DS1621_START_CONVERT;
  if(write(fd, &byte, 1) != 1) {
    perror("write");
  }


  for(;;) {
    // read temperature in two separated transfers
    // first write register address to the slave
    byte = DS1621_READ_TEMP;
    if(write(fd, &byte, 1) != 1) {
      perror("write");
    }

    // and then read value from slave
    uint8_t rx[2];
    if(read(fd, rx, 2) != 2) {
      perror("read");
    }

    //			printf("rx: %d %d\n", rx[0], rx[1]);

    uint8_t	count_per_c = i2c_read_byte(fd, DS1621_ADDR, DS1621_READ_SLOPE);
    uint8_t	count_remain = i2c_read_byte(fd, DS1621_ADDR, DS1621_READ_COUNTER);

    float temperature = rx[0] - 0.25 + ((float) count_per_c - count_remain) / count_per_c;
    printf("%f\n", temperature);
  }
  return 0;
}
