#include <stdio.h>
#include <math.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define ADXL345_ID             0x00
#define ADXL345_POWER_CTL      0x2D
#define ADXL345_DATA_FORMAT    0x31
#define ADXL345_DATAX0         0x32
#define ADXL345_ADDR           0x53

#define ADXL345_READ_CMD       0x80
#define ADXL345_MULTI_CMD      0x40

SemaphoreHandle_t transfer_done;


void BOARD_InitPins(void) {
  // SPI alternate function
  // ECSPI1_MOSI - GPIO5_IO07
  // ECSPI1_MISO - GPIO5_IO08
  // ECSPI1_CLK  - GPIO5_IO6
  // ECSPI1_SS0  - GPIO5_IO9
  IOMUXC_SetPinMux(IOMUXC_ECSPI1_MISO_ECSPI1_MISO, 0U);
  IOMUXC_SetPinMux(IOMUXC_ECSPI1_MOSI_ECSPI1_MOSI, 0U);
  IOMUXC_SetPinMux(IOMUXC_ECSPI1_SCLK_ECSPI1_SCLK, 0U);
  IOMUXC_SetPinMux(IOMUXC_ECSPI1_SS0_ECSPI1_SS0, 0U);
}

// interrupts on Transfer Complete only
void ECSPI1_IRQHandler() {
  BaseType_t woken;
  // clear Transfer Complete flag
  ECSPI1->STATREG |= ECSPI_STATREG_TC_MASK;

  // wake up task
  xSemaphoreGiveFromISR(transfer_done, &woken);

  // context switch to higher priority task
  portYIELD_FROM_ISR(woken);
}

void adxl345_set(uint8_t reg, uint8_t val) {
  uint32_t tx = ((reg & 0x3F) << 8) | val;
  ecspi_transfer_t masterXfer;
  masterXfer.txData = &tx;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = 1;
  masterXfer.channel = kECSPI_Channel0;
  status_t err = ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);
  configASSERT(err == kStatus_Success);
}

uint8_t adxl345_get(uint8_t reg) {
  uint32_t tx = ((reg & 0x3F | ADXL345_READ_CMD) << 8);
  uint32_t rx = -1;
  ecspi_transfer_t masterXfer;
  masterXfer.txData = &tx;
  masterXfer.rxData = &rx;
  masterXfer.dataSize = 1;
  masterXfer.channel = kECSPI_Channel0;
  status_t err = ECSPI_MasterTransferBlocking(ECSPI1, &masterXfer);
  configASSERT(err == kStatus_Success);

  return rx & 0xFF;
}

void adxl345_get_axis(uint16_t *x, uint16_t *y, uint16_t *z) {
  // transmit 8*7 bits
  // first byte - register address
  // remaining dummy bytes for receive DATAX0, DATAX1, ..., DATAZ1

  // set new burst size
  ECSPI1->CONREG = (ECSPI1->CONREG & ~ECSPI_CONREG_BURST_LENGTH_MASK)
    | ECSPI_CONREG_BURST_LENGTH(8 * 7 - 1);


  // put two 32bits to FIFO buffer
  ECSPI1->TXDATA = (ADXL345_READ_CMD | ADXL345_MULTI_CMD | ADXL345_DATAX0) << 16;
  ECSPI1->TXDATA = 0x00;

  // wait for Transfer Complete interrupt
  xSemaphoreTake(transfer_done, portMAX_DELAY);

  // data are received in this format:
  // X0, X1
  // Y0, Y1, Z0, Z1
  volatile uint32_t r = ECSPI1->RXDATA;
  *x = ((r & 0xFF00) >> 8) | ((r & 0xFF) << 8);
  r = ECSPI1->RXDATA;
  *y = ((r & 0xFF000000) >> 24) | ((r & 0xFF0000) >> 8);
  *z = ((r & 0xFF00) >> 8) | ((r & 0xFF) << 8);
}

void measure_task(void* arg) {
  CLOCK_EnableClock(kCLOCK_Ecspi1);
  CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI2 source to SYSTEM PLL1 800MHZ */
  CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */

  ecspi_master_config_t masterConfig;
  ECSPI_MasterGetDefaultConfig(&masterConfig);
  masterConfig.baudRate_Bps = 2000000U;
  masterConfig.burstLength = 16;
  masterConfig.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
  masterConfig.channelConfig.phase = kECSPI_ClockPhaseSecondEdge;
  masterConfig.channelConfig.clockInactiveState = kECSPI_ClockInactiveStateHigh;
  masterConfig.channelConfig.chipSlectActiveState = kECSPI_ChipSelectActiveStateLow;
  masterConfig.channelConfig.waveForm = kECSPI_WaveFormSingle;
  masterConfig.channelConfig.polarity = kECSPI_PolarityActiveLow;
  masterConfig.channelConfig.phase = kECSPI_ClockPhaseSecondEdge;

  uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootEcspi1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootEcspi1));

  ECSPI_MasterInit(ECSPI1, &masterConfig, clk);

  // check we are talking with ADXL345
  uint16_t id = adxl345_get(ADXL345_ID);
  if(id != 0b11100101) {
    printf("adxl not found %x\r\n", id);
    for(;;);
  }

  adxl345_set(ADXL345_POWER_CTL, 0);
  adxl345_set(ADXL345_POWER_CTL, 16);
  adxl345_set(ADXL345_POWER_CTL, 8);

  uint16_t data_format = adxl345_get(ADXL345_DATA_FORMAT);
  uint16_t g_range = pow(2, (data_format & 0x3) + 1);
  uint16_t resolution = (data_format & (1 << 3)) ? 16 : 10;
  float scale = pow(2, resolution - 1);

  printf("DATA_FORMAT = %x\r\n", data_format);
  printf("grange = +- %d\r\n", g_range);
  printf("resolution = %dbits, scale = %f\r\n", resolution, scale);


  // enable IRQ and set it up
  EnableIRQ(ECSPI1_IRQn);
  NVIC_SetPriority(ECSPI1_IRQn, 2);
  ECSPI_EnableInterrupts(ECSPI1, kECSPI_TransferCompleteInterruptEnable);

  for(;;) {
    int16_t x, y, z;
    adxl345_get_axis(&x, &y, &z);

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

int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();
  BOARD_InitMemory();

  transfer_done = xSemaphoreCreateBinary();
  configASSERT(transfer_done);

  if (xTaskCreate(measure_task, "app", 512, NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
    printf("\r\nFailed to create application task\r\n");
    for(;;);
  }

  vTaskStartScheduler();
  printf("Failed to start FreeRTOS.\n");
  for(;;);
}
