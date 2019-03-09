#include <stdio.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "pin_mux.h"
#include "clock_config.h"

#define ADXL345_POWER_CTL      0x2D
#define ADXL345_DATA_FORMAT    0x31
#define ADXL345_DATAX0         0x32
#define ADXL345_ADDR           0x53

void BOARD_InitPins(void) {
  IOMUXC_SetPinMux(IOMUXC_I2C2_SCL_I2C2_SCL, 1U);
  IOMUXC_SetPinConfig(IOMUXC_I2C2_SCL_I2C2_SCL,
      IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
      IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
  IOMUXC_SetPinMux(IOMUXC_I2C2_SDA_I2C2_SDA, 1U);
  IOMUXC_SetPinConfig(IOMUXC_I2C2_SDA_I2C2_SDA,
      IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
      IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
      IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
}

void i2c_transmit(i2c_rtos_handle_t* i2c, i2c_master_transfer_t *xfer, const char* file, int line) {
  int tries = 0;

  // i2c bus is still busy after previous transmission, so we have to keep trying
  while(I2C_RTOS_Transfer(i2c, xfer) != kStatus_Success) {
    // I2C_MasterTransferNonBlocking() call will lock i2c internal state to kCheckAddressState  
    // if i2c bus is BUSY, so reset it before each try
    i2c->drv_handle.state = 0;
    tries++;

    if(tries == 10) {
      printf("too long while accessing i2c device at %s:%d\r\n", file, line);
    }
  }
}

void i2c_read(i2c_rtos_handle_t* i2c, uint8_t reg, uint8_t *data, int size) {
  i2c_master_transfer_t masterXfer;
  masterXfer.slaveAddress = ADXL345_ADDR;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = reg;
  masterXfer.subaddressSize = 1;
  masterXfer.data = data;
  masterXfer.dataSize = size;
  masterXfer.flags = kI2C_TransferDefaultFlag;
  i2c_transmit(i2c, &masterXfer, __FILE__, __LINE__);
}

uint8_t i2c_read_byte(i2c_rtos_handle_t* i2c, uint8_t reg) {
  uint8_t rx;
  i2c_read(i2c, reg, &rx, 1);
  return rx;
}

void i2c_write(i2c_rtos_handle_t* i2c, uint8_t reg, uint8_t val) {
  i2c_master_transfer_t masterXfer;
  masterXfer.slaveAddress = ADXL345_ADDR;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = reg;
  masterXfer.subaddressSize = 1;
  masterXfer.data = &val;
  masterXfer.dataSize = 1;
  masterXfer.flags = kI2C_TransferDefaultFlag;

  i2c_transmit(i2c, &masterXfer, __FILE__, __LINE__);
}

void measure_task(void*) {
  i2c_rtos_handle_t i2c;

  CLOCK_SetRootMux(kCLOCK_RootI2c2, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
  CLOCK_SetRootDivider(kCLOCK_RootI2c2, 1U, 4U);                /* Set root clock to 160MHZ / 4 = 40MHZ */

  uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootI2c2)) / (CLOCK_GetRootPostDivider(kCLOCK_RootI2c2)) / 5 /* SYSTEM PLL1 DIV5 */;

  i2c_master_config_t conf;
  I2C_MasterGetDefaultConfig(&conf);
  conf.baudRate_Bps = 100000U;
  configASSERT(I2C_RTOS_Init(&i2c, I2C2, &conf, clk) == kStatus_Success);

  NVIC_SetPriority(I2C2_IRQn, 2);

  i2c_write(&i2c, ADXL345_POWER_CTL, 0);
  i2c_write(&i2c, ADXL345_POWER_CTL, 16);
  i2c_write(&i2c, ADXL345_POWER_CTL, 8);

  uint16_t data_format = i2c_read_byte(&i2c, ADXL345_DATA_FORMAT);
  uint16_t g_range = pow(2, (data_format & 0x3) + 1);
  uint16_t resolution = (data_format & (1 << 3)) ? 16 : 10;
  float scale = pow(2, resolution - 1);

  printf("DATA_FORMAT = %x\r\n", data_format);
  printf("grange = +- %d\r\n", g_range);
  printf("resolution = %dbits, scale = %f\r\n", resolution, scale);

  for(;;) {
    uint8_t data[6];
    i2c_read(&i2c, ADXL345_DATAX0, data, sizeof(data));

    int16_t x = (((int) data[1]) << 8) | data[0];
    int16_t y = (((int) data[3]) << 8) | data[2];
    int16_t z = (((int) data[5]) << 8) | data[4];

    float g = 9.81;
    printf(
        "%6.3f %6.3f %6.3f\r\n",
        g_range * x * g / scale,
        g_range * y * g / scale,
        g_range * z * g / scale
    );

    vTaskDelay(50);
  }
}

void main() {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("I2C2 adxl345 example started\r\n");

  xTaskCreate(measure_task, "task", 256, NULL, 0, NULL);
  vTaskStartScheduler();

  for(;;);
}
