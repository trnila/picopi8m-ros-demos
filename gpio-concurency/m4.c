#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
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

    IOMUXC_SetPinMux(IOMUXC_SAI2_RXD0_GPIO4_IO23, 0U);
    IOMUXC_SetPinConfig(IOMUXC_SAI2_RXD0_GPIO4_IO23,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
}


int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("GPIO blink example started\r\n");

    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 0;
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(GPIO4, 23, &conf);

    int state = 0;
    for(;;) {
      GPIO_PinWrite(GPIO4, 23, state);
      state = !state;
    }
}
