#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <limits.h>
#include <errno.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define ADXL345_ID             0x00U
#define ADXL345_POWER_CTL      0x2DU
#define ADXL345_DATA_FORMAT    0x31U
#define ADXL345_DATAX0         0x32U
#define ADXL345_ADDR           0x53U

#define ADXL345_READ_CMD       0x80U
#define ADXL345_MULTI_CMD      0x40U

void adxl345_get_axis(int spi_fd, int16_t *x, int16_t *y, int16_t *z) {
  /*
   * send two 32bit transactions
   * each transaction starts with register address follwing 3 bytes of data
   */
  uint32_t tx[] = {
    (ADXL345_READ_CMD | ADXL345_MULTI_CMD | ADXL345_DATAX0) << 24,
    (ADXL345_READ_CMD | ADXL345_MULTI_CMD | ADXL345_DATAX0 + 3) << 24,
  };

  uint8_t rx[sizeof(tx)];

  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long) tx;
  tr.rx_buf = (unsigned long) rx;
  tr.len = sizeof(tx);
  tr.delay_usecs = 0;
  tr.speed_hz = 2000000;
  tr.bits_per_word = 32; // send transaction of 32 bits, unfortunelly we can set anything but 8, 16, 32

  if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) != sizeof(tx)) {
    perror("spi transfer ioctl: ");
    exit(1);
  }

  /**
   * received data are filled in this layout:
   *   0    1    2    3
   * | Y0 | X1 | X0 | - |
   * | Z1 | Z0 | Y1 | - |
   *   4    5    6    7
   */
  *x = rx[2] | (rx[1] << 8);
  *y = rx[0] | (rx[6] << 8);
  *z = rx[5] | (rx[4] << 8);
}

void adxl345_set(int spi_fd, uint8_t reg, uint8_t val) {
  uint16_t tx = ((reg & 0x3F) << 8) | val;

  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long) &tx;
  tr.rx_buf = (unsigned long) NULL;
  tr.len = sizeof(tx);
  tr.delay_usecs = 0;
  tr.speed_hz = 2000000;
  tr.bits_per_word = sizeof(tx) * 8;

  if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) != sizeof(tx)) {
    perror("spi transfer ioctl: ");
    exit(1);
  }
}

uint8_t adxl345_get(int spi_fd, uint8_t reg) {
  uint16_t tx = (((reg & 0x3F) | ADXL345_READ_CMD) << 8);
  uint16_t rx = -1;

  struct spi_ioc_transfer tr{};
  tr.tx_buf = (unsigned long) &tx;
  tr.rx_buf = (unsigned long) &rx;
  tr.len = sizeof(tx);
  tr.delay_usecs = 0;
  tr.speed_hz = 2000000;
  tr.bits_per_word = sizeof(tx) * 8;

  if(ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) != sizeof(tx)) {
    perror("spi transfer ioctl: ");
    exit(1);
  }
  return rx & 0xFF;
}

int spi_init() {
  int spi_fd = open("/dev/spidev1.0", O_RDWR);
  if(spi_fd < 0) {
    perror("open spi");
    exit(1);
  }

  uint8_t mode = SPI_CPHA | SPI_CPOL;
  if(ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) != 0) {
    perror("ioctl mode ");
    exit(1);
  }

  uint8_t bits_per_word = 32;
  if(ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) != 0) {
    perror("ioctl bits_per_word: ");
    exit(1);
  }

  return spi_fd;
}

int main(int argc, char **argv) {
  int spi_fd = spi_init();

  // check we are talking with ADXL345
  uint16_t id = adxl345_get(spi_fd, ADXL345_ID);
  if(id != 0b11100101) {
    printf("adxl not found %x\n", id);
    exit(1);
  }

  adxl345_set(spi_fd, ADXL345_POWER_CTL, 0);
  adxl345_set(spi_fd, ADXL345_POWER_CTL, 16);
  adxl345_set(spi_fd, ADXL345_POWER_CTL, 8);

  uint16_t data_format = adxl345_get(spi_fd, ADXL345_DATA_FORMAT);
  uint16_t g_range = pow(2, (data_format & 0x3) + 1);
  uint16_t resolution = (data_format & (1 << 3)) ? 16 : 10;
  float scale = pow(2, resolution - 1);

  printf("DATA_FORMAT = %x\n", data_format);
  printf("grange = +- %d\n", g_range);
  printf("resolution = %dbits, scale = %f\n", resolution, scale);

  for(;;) {
    int16_t x, y, z;
    adxl345_get_axis(spi_fd, &x, &y, &z);

    float g = 9.81;
    printf(
        "%6.3f %6.3f %6.3f\n",
        g_range * x * g / scale,
        g_range * y * g / scale,
        g_range * z * g / scale
        );
    usleep(50 * 1000);
  }

  return 0;
}
