# Accelerometer ADXL345 over SPI
## Connection
|     PICO-PI-IMX8M     | SEN0032 |
|:---------------------:|:-------:|
|          GND          |   GND   |
|          3V3          |   VCC   |
|  ECSPI1_SS0/GPIO5_IO9 |    CS   |
|                       |   INT1  |
|                       |   INT2  |
| ECSPI1_MISO/GPIO5_IO8 |   SDO   |
| ECSPI1_MOSI/GPIO5_IO7 |   SDA   |
| ECSPI1_SCLK/GPIO5_IO6 |   SCL   |
