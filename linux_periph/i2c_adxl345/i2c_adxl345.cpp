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
#include <math.h>

#define ADXL345_POWER_CTL      0x2D
#define ADXL345_DATA_FORMAT    0x31
#define ADXL345_DATAX0         0x32
#define ADXL345_ADDR           0x53

/*
 * read from slave in single I2C transaction with repeated start
 */
void i2c_read(int fd, uint8_t reg, uint8_t *data, int count) {
  // we will make two transfers in single transaction
  struct i2c_msg msgs[2];
  struct i2c_rdwr_ioctl_data xfer;
  xfer.msgs = msgs;
  xfer.nmsgs = 2;

  // in first transaction we will send register address
  msgs[0].addr = ADXL345_ADDR;
  msgs[0].flags = 0;
  msgs[0].buf = &reg;
  msgs[0].len = sizeof(reg);

  // in second transaction we will read bytes from slave
  msgs[1].addr = ADXL345_ADDR;
  msgs[1].flags = I2C_M_RD;
  msgs[1].buf = data;
  msgs[1].len = count;

  // make transfer
  if(ioctl(fd, I2C_RDWR, &xfer) < 0) {
    perror("ioctl");
    exit(1);
  }
}

uint8_t i2c_read_byte(int fd, uint8_t reg) {
  uint8_t rx;
  i2c_read(fd, reg, &rx, sizeof(rx));
  return rx;
}

void i2c_write(int fd, uint8_t reg, uint8_t val) {
    uint8_t data[] = {reg, val};
    if(write(fd, &data, sizeof(data)) != sizeof(data)) {
      perror("write");
      exit(1);
    }
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

  // set slave address when using simplex - ie read, write
  if(ioctl(fd, I2C_SLAVE, ADXL345_ADDR) < 0) {
    perror("ioctl");
    exit(1);
  }

  i2c_write(fd, ADXL345_POWER_CTL, 0);
  i2c_write(fd, ADXL345_POWER_CTL, 16);
  i2c_write(fd, ADXL345_POWER_CTL, 8);

  uint16_t data_format = i2c_read_byte(fd, ADXL345_DATA_FORMAT);
  uint16_t g_range = pow(2, (data_format & 0x3) + 1);
  uint16_t resolution = (data_format & (1 << 3)) ? 16 : 10;
  float scale = pow(2, resolution - 1);

  printf("DATA_FORMAT = %x\n", data_format);
  printf("grange = +- %d\n", g_range);
  printf("resolution = %dbits, scale = %f\n", resolution, scale);

  for(;;) {
    uint8_t data[6];
    i2c_read(fd, ADXL345_DATAX0, data, sizeof(data));

    int16_t x = (((int) data[1]) << 8) | data[0];
    int16_t y = (((int) data[3]) << 8) | data[2];
    int16_t z = (((int) data[5]) << 8) | data[4];

    float g = 9.81;
    printf(
        "%6.3f %6.3f %6.3f\n",
        g_range * x * g / scale,
        g_range * y * g / scale,
        g_range * z * g / scale
    );
    usleep(50000);
  }

  return 0;
}
