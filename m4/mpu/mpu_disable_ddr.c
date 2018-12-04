#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_rdc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

void BOARD_InitPins(void) {
  IOMUXC_SetPinMux(IOMUXC_UART3_RXD_UART3_RX, 0U);
  IOMUXC_SetPinConfig(IOMUXC_UART3_RXD_UART3_RX, 
      IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
      IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
  IOMUXC_SetPinMux(IOMUXC_UART3_TXD_UART3_TX, 0U);
  IOMUXC_SetPinConfig(IOMUXC_UART3_TXD_UART3_TX, 
      IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
      IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
      IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);

  IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U);
}

void HardFault_Handler(void) {
  printf("hard fault\r\n");
  for(;;);
}


void MemManage_Handler(void) {
  // [0] - fetch from non-executable location
  // [1] - load/store from non-accessible location
  uint32_t cfsr  = SCB->CFSR;

  uint32_t mmfar = SCB->MMFAR; // address of location that generated memmanage fault
  uint32_t bfar  = SCB->BFAR;  // address of location that generated busfault

  printf("MemManage fault\r\n");
  printf("cfsr = %x\r\n", cfsr);
  printf("mmfar = %x\r\n", mmfar);
  printf("bfar = %x\r\n", bfar);
  for(;;);
}


volatile int x;

int main(void) {
  BOARD_RdcInit();
  BOARD_InitPins();
  BOARD_BootClockRUN();
  BOARD_InitDebugConsole();

  // before MPU setup we should successfully access any location
  x = *((uint32_t*) 0x40000000);
  GPIO_PinWrite(GPIO4, 26, 1);


  __DMB();
  /* Disable the MPU until we fully configure it */
  MPU->CTRL = 0;

  /* configure full access/0x3 (AP) from 0x2000_0000 to 0x4000_0000 (~536 Mbytes - 2^29) */
  MPU->RBAR = (0x20000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (0 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (29 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* configure full aceess to code - 256 kbytes - 2^8 */
  MPU->RBAR = (0x1FFF0000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (1 << MPU_RBAR_REGION_Pos);
  MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (8 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* configure full aceess to DDR (0x4000_0000 to 0xC000_0000) - ~2 GB - 2^31
     uncomment to access linux memory from M4 */
  //MPU->RBAR = (0x40000000U & MPU_RBAR_ADDR_Msk) | MPU_RBAR_VALID_Msk | (2 << MPU_RBAR_REGION_Pos);
  //MPU->RASR = (0x3 << MPU_RASR_AP_Pos) | (31 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

  /* enable MemManage handlers */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

  /* enable mpu and use default cortex-m map in privileged mode */
  MPU->CTRL = MPU_CTRL_ENABLE_Msk;

  /* Memory barriers to ensure subsequence data & instruction
   * transfers using updated MPU settings.
   */
  __DSB();
  __ISB();

  // we should successfully access peripherals
  GPIO_PinWrite(GPIO4, 26, 1);
  printf("Peripheral accessed...\r\n");

  // but not DDR memory
  x = *((uint32_t*) 0x40000000);

  printf("Failed: could access DDR memory\r\n");
  for(;;);
}
