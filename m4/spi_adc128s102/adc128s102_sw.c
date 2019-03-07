#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_ecspi.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

#define PIN_IN 8
#define PIN_OUT 7
#define PIN_CLK 6
#define PIN_SS 9

void BOARD_InitPins(void) {
    // SPI alternate function
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MISO_GPIO5_IO08, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_MISO_GPIO5_IO08, 
	    IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
	    IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_MOSI_GPIO5_IO07, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_MOSI_GPIO5_IO07, 
	    IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
	    IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_SCLK_GPIO5_IO06, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_SCLK_GPIO5_IO06, 
	    IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
	    IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
    IOMUXC_SetPinMux(IOMUXC_ECSPI1_SS0_GPIO5_IO09, 0U);
    IOMUXC_SetPinConfig(IOMUXC_ECSPI1_SS0_GPIO5_IO09, 
	    IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
	    IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
}

/**
 * example demonstrates software SPI
 * keep in mind that CLOCK is just ~730 Khz
 */
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    gpio_pin_config_t conf;
    conf.direction = kGPIO_DigitalOutput;
    conf.outputLogic = 1; // idle state is HIGH
    conf.interruptMode = kGPIO_NoIntmode;
    GPIO_PinInit(GPIO5, PIN_OUT, &conf);
    GPIO_PinInit(GPIO5, PIN_CLK, &conf);
    GPIO_PinInit(GPIO5, PIN_SS, &conf);

    conf.direction = kGPIO_DigitalInput;
    GPIO_PinInit(GPIO5, PIN_IN, &conf);

    for(;;) {
	GPIO_PinWrite(GPIO5, PIN_SS, 0);
	uint32_t txs[] = {
	    (1 << (8 + 3 + 16)) | (2 << (8 + 3)),
	    (3 << (8 + 3 + 16)) | (4 << (8 + 3)),
	    (5 << (8 + 3 + 16)) | (6 << (8 + 3)),
	    (7 << (8 + 3 + 16)) | (0 << (8 + 3)),
	};	
	uint32_t rxs[4];

	for(int i = 0; i < sizeof(txs) / sizeof(*txs); i++) {
	    uint32_t tx = txs[i];
	    rxs[i] = 0;

	    for(uint32_t bit = 0x80000000; bit; bit >>= 1) {
		GPIO_PinWrite(GPIO5, PIN_OUT, (tx & bit) > 0);

		GPIO_PinWrite(GPIO5, PIN_CLK, 0);

		rxs[i] <<= 1;
		rxs[i] |= GPIO_PinRead(GPIO5, PIN_IN);

		GPIO_PinWrite(GPIO5, PIN_CLK, 1);
	    }
	}


	GPIO_PinWrite(GPIO5, PIN_SS, 1);

	for(int i = 0; i < sizeof(txs) / sizeof(*txs); i++) {
	    uint32_t val = rxs[i];
	    printf("%5d %5d ", (val & 0xFFF0000U) >> 16U, val & 0xFFF);
	}
	printf("\r\n");

	for(int i = 0; i < 240000; i++) asm("nop");
    }
    for(;;);
}
