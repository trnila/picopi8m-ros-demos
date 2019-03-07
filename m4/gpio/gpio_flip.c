#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

#ifndef VERSION
#define VERSION 0
#endif

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U);
}


int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("GPIO flip measurement started %X %X\r\n");

    // setup GPIO4_IO26 (122) as output
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(GPIO4, 26, &conf);

#if VERSION == 0
    // 167 ns
    asm(
        "mov r0, 0\n"           // no pin high
        "mov r2, #0x4000000\n " // pin 26 high, 1 << 26
        "ldr r3, .vals\n"       // load GPIO4->DR
        ".loop:\n"
        "str r0, [r3]\n"        // set pin high 
        "str r2, [r3]\n"        // set pin low
        "b .loop\n"
        ".vals:\n"
        ".word 0x30230000"      // GPIO4->DR
    );
#elif VERSION == 1
    // 167 ns
    for(;;) {
        GPIO4->DR = 0x4000000;
        GPIO4->DR = 0;
    }
#elif VERSION == 2
    // 333 ns
    for(;;) {
        GPIO4->DR |= 0x4000000;
        GPIO4->DR &= ~0x4000000;
    }
#elif VERSION == 3
    // 417 ns
    for(;;) {
        GPIO_PinWrite(GPIO4, 26, 1);
        GPIO_PinWrite(GPIO4, 26, 0);
    }
#endif

    printf("example failed");
    for(;;);
}
