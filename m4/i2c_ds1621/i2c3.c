#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"

#define DS1621_ADDR          0x48

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

    // set alternate function I2C3 on 147, 146 pins
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_I2C3_SCL, 0U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SCL_I2C3_SCL,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_VSEL(4U));
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_I2C3_SDA, 0U);
    IOMUXC_SetPinConfig(IOMUXC_I2C3_SDA_I2C3_SDA,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(2U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);

}

void keep_reading(I2C_Type* i2c, uint8_t addr);

// TODO: i2c3 currently not working, SDA/SCL stays LOW at the start of communication
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("I2C3 ds1621 example started\r\n");

    CLOCK_EnableClock(kCLOCK_I2c3);
    CLOCK_SetRootMux(kCLOCK_RootI2c3, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootI2c3, 1U, 4U);                /* Set root clock to 160MHZ / 4 = 40MHZ */

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootI2c3)) / (CLOCK_GetRootPostDivider(kCLOCK_RootI2c3)) / 5 /* SYSTEM PLL1 DIV5 */;

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 100000U;
    I2C_MasterInit(I2C3, &masterConfig, clk);

		keep_reading(I2C3, DS1621_ADDR);

    for(;;);
}
