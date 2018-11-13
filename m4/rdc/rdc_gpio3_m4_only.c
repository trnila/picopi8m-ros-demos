#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h"
#include "fsl_rdc.h"
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
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK); // enable pull-up to prevent pin floating
}

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("GPIO3 rdc with linux domain readonly\r\n");

    rdc_periph_access_config_t access_conf;
    access_conf.periph = kRDC_Periph_GPIO4;
    access_conf.lock = false;
    access_conf.enableSema = false;
    access_conf.policy = RDC_ACCESS_POLICY(0, kRDC_ReadOnly) | // Quad A53 (Linux) domain
                  RDC_ACCESS_POLICY(1, kRDC_ReadWrite) |       // M4 domain
                  RDC_ACCESS_POLICY(2, kRDC_ReadWrite) |
                  RDC_ACCESS_POLICY(3, kRDC_ReadWrite);

    RDC_SetPeriphAccessConfig(RDC, &access_conf);
    

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

      printf("%d\r\n", state);

      // some delay
      for(int i = 0; i < 6400000; i++) {
        asm("nop");
      }
    }
}
