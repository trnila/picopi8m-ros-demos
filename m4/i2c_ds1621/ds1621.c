#include <stdio.h>
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_i2c.h"
#include "ds1621.h"


void i2c_write_byte(I2C_Type* i2c, uint8_t addr, uint8_t data) {
    i2c_master_transfer_t masterXfer;
    masterXfer.slaveAddress = addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &data;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;
    while(I2C_MasterTransferBlocking(i2c, &masterXfer) != 0);
}

void i2c_read(I2C_Type* i2c, uint8_t addr, uint8_t reg, uint8_t *data, int size) {
    i2c_master_transfer_t masterXfer;
    masterXfer.slaveAddress = addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;
    while(I2C_MasterTransferBlocking(i2c, &masterXfer) != 0);

    masterXfer.slaveAddress = addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = data;
    masterXfer.dataSize = size;
    masterXfer.flags = kI2C_TransferRepeatedStartFlag;
    while(I2C_MasterTransferBlocking(i2c, &masterXfer) != 0);
}

uint8_t i2c_read_byte(I2C_Type* i2c, uint8_t addr, uint8_t reg) {
    uint8_t recv = 0;
    i2c_read(i2c, addr, reg, &recv, 1);
    return recv;
}


void keep_reading(I2C_Type* i2c, uint8_t addr) {
  i2c_write_byte(i2c, addr, DS1621_START_CONVERT);

  uint8_t raw[2];
  for(;;) {
    i2c_read(i2c, addr, DS1621_READ_TEMP, raw, 2);
    uint8_t count_per_c = i2c_read_byte(i2c, addr, DS1621_READ_SLOPE);
    uint8_t count_remain = i2c_read_byte(i2c, addr, DS1621_READ_COUNTER);

    float precise = (float) raw[0] - 0.25 + (float) (count_per_c - count_remain) / count_per_c;

    printf("%d.%c %d.%d%d%d\r\n",
      raw[0], raw[1] & 0x80 ? '5' : '0',
      (int) precise, (int) (precise * 10) % 10, (int) (precise * 100) % 10, (int) (precise * 1000) % 10
    );
  }
}
