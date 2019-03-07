#include <stdio.h>
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
#include "ds1621.h"


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

void i2c_read(i2c_rtos_handle_t* i2c, uint8_t addr, uint8_t reg, uint8_t *data, int size) {
  i2c_master_transfer_t masterXfer;
  masterXfer.slaveAddress = addr;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = reg;
  masterXfer.subaddressSize = 1;
  masterXfer.data = data;
  masterXfer.dataSize = size;
  masterXfer.flags = kI2C_TransferDefaultFlag;
  i2c_transmit(i2c, &masterXfer, __FILE__, __LINE__);
}

void i2c_write_byte(i2c_rtos_handle_t* i2c, uint8_t addr, uint8_t data) {
  i2c_master_transfer_t masterXfer;
  masterXfer.slaveAddress = addr;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = (uint32_t) NULL;
  masterXfer.subaddressSize = 0;
  masterXfer.data = &data;
  masterXfer.dataSize = 1;
  masterXfer.flags = kI2C_TransferDefaultFlag;

  i2c_transmit(i2c, &masterXfer, __FILE__, __LINE__);
}

uint8_t i2c_read_byte(i2c_rtos_handle_t* i2c, uint8_t addr, uint8_t reg) {
  uint8_t recv = 0;
  i2c_read(i2c, addr, reg, &recv, 1);
  return recv;
}

void task() {
  i2c_rtos_handle_t i2c;

  CLOCK_SetRootMux(kCLOCK_RootI2c2, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
  CLOCK_SetRootDivider(kCLOCK_RootI2c2, 1U, 4U);                /* Set root clock to 160MHZ / 4 = 40MHZ */

  uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootI2c2)) / (CLOCK_GetRootPostDivider(kCLOCK_RootI2c2)) / 5 /* SYSTEM PLL1 DIV5 */;

  i2c_master_config_t conf;
  I2C_MasterGetDefaultConfig(&conf);
  conf.baudRate_Bps = 100000U;
  configASSERT(I2C_RTOS_Init(&i2c, I2C2, &conf, clk) == kStatus_Success);

  NVIC_SetPriority(I2C2_IRQn, 2);

  uint8_t raw[2];
  int addr = DS1621_ADDR;
  i2c_write_byte(&i2c, addr, DS1621_START_CONVERT);
  for(;;) {
    i2c_read(&i2c, addr, DS1621_READ_TEMP, raw, 2);
    uint8_t count_per_c = i2c_read_byte(&i2c, addr, DS1621_READ_SLOPE);
    uint8_t count_remain = i2c_read_byte(&i2c, addr, DS1621_READ_COUNTER);

    float precise = (float) raw[0] - 0.25 + (float) (count_per_c - count_remain) / count_per_c;

    printf("%d.%c %d.%d%d%d\r\n",
        raw[0], raw[1] & 0x80 ? '5' : '0',
        (int) precise, (int) (precise * 10) % 10, (int) (precise * 100) % 10, (int) (precise * 1000) % 10
        );

    vTaskDelay(1000);
  }
}

int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  printf("I2C2 ds1621 freertos example started\r\n");

  xTaskCreate(task, "task", 256, NULL, 0, NULL);
  vTaskStartScheduler();

  for(;;);
}
