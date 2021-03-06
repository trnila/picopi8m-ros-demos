#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U); // 122
}


int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("GPIO blink example started\r\n");

    // setup GPIO4_IO26 (122) as output
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(GPIO4, 26, &conf);

    int state = 0;
    for(;;) {
      GPIO_PinWrite(GPIO4, 26, state);
      state = !state;

      // some delay
      for(int i = 0; i < 6400000; i++) {
        asm("nop");
      }
    }
}
