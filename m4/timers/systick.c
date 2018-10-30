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

    IOMUXC_SetPinMux(IOMUXC_SAI2_TXD0_GPIO4_IO26, 0U); // 122
    IOMUXC_SetPinConfig(IOMUXC_SAI2_TXD0_GPIO4_IO26, 
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) | // driver strength = 45 ohm
                        IOMUXC_SW_PAD_CTL_PAD_SRE(2U) | // slew rate - fast
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
}

volatile uint32_t tick;

void SysTick_Handler() {
  if(tick & 1) {
    GPIO_WritePinOutput(GPIO4, 26, 1);
  } else {
    GPIO_WritePinOutput(GPIO4, 26, 0);
  }
  tick++;
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("Systick example started\r\n");
    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;;
    GPIO_PinInit(GPIO4, 26, &conf);

    uint32_t period_us = 20000;
    uint32_t ticks = SystemCoreClock / 1000000UL * period_us;

    printf("SystemCoreClock = %u\r\n", SystemCoreClock);
    printf("ticks = %u\r\n", ticks);
    SysTick_Config(ticks);

    for(;;);
}
