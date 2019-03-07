#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U); // 122
    IOMUXC_SetPinConfig(IOMUXC_SAI2_TXD0_GPIO4_IO26, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) | // driver strength = 45 ohm
                        IOMUXC_SW_PAD_CTL_PAD_SRE(2U) | // slew rate - fast
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK); // enable pull-up to prevent pin floating
}

const int PIN = 26;
volatile int falling_edge_count;

void GPIO4_Combined_16_31_IRQHandler() {
  GPIO_PortClearInterruptFlags(GPIO4, 1 << PIN); // GPIO4->ISR = 1 << PIN;
  falling_edge_count++;
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("GPIO falling edge irq example started\r\n");

    // setup GPIO4_IO26 (122) as input
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalInput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_IntFallingEdge;
    GPIO_PinInit(GPIO4, PIN, &conf);
    GPIO_EnableInterrupts(GPIO4, 1 << PIN);
    EnableIRQ(GPIO4_Combined_16_31_IRQn);

    for(;;) {
      printf("total irqs %d\r\n", falling_edge_count);

      // some delay
      for(int i = 0; i < 6400000; i++) {
        asm("nop");
      }
    }
}
