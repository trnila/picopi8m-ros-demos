#include "fsl_iomuxc.h"

#define CHANNEL(n) (((n) & 7) << (8+3))
#define SAMPLE(a, b) (CHANNEL(b) | (CHANNEL(a) << 16))
#define SAMPLE_A(val) (((val) & 0xFFFF000) >> 16)
#define SAMPLE_B(val) ((val) & 0xFFFF) 

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
