#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "ds1621.h"

void BOARD_InitPins(void) {
    IOMUXC_SetPinMux(IOMUXC_I2C2_SCL_I2C2_SCL, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C2_SCL_I2C2_SCL,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK);
    IOMUXC_SetPinMux(IOMUXC_I2C2_SDA_I2C2_SDA, 1U);
    IOMUXC_SetPinConfig(IOMUXC_I2C2_SDA_I2C2_SDA,
                        IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
                        IOMUXC_SW_PAD_CTL_PAD_SRE(1U) |
                        IOMUXC_SW_PAD_CTL_PAD_ODE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_PUE_MASK |
                        IOMUXC_SW_PAD_CTL_PAD_HYS_MASK);
}

void keep_reading(I2C_Type* i2c, uint8_t addr);

int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("I2C2 ds1621 example started\r\n");

    CLOCK_SetRootMux(kCLOCK_RootI2c2, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootI2c2, 1U, 4U);                /* Set root clock to 160MHZ / 4 = 40MHZ */

    uint32_t clk = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootI2c2)) / (CLOCK_GetRootPostDivider(kCLOCK_RootI2c2)) / 5 /* SYSTEM PLL1 DIV5 */;

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = 100000U;
    I2C_MasterInit(I2C2, &masterConfig, clk);

    keep_reading(I2C2, DS1621_ADDR);

    for(;;);
}
