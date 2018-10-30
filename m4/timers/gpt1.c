#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_gpt.h"
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

void GPT1_IRQHandler() {
  GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);

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

    printf("GPT1 example started\r\n");

    uint32_t period_us = 20000;
    uint32_t divider = 1;

    CLOCK_SetRootMux(kCLOCK_RootGpt1, kCLOCK_GptRootmuxSysPll1Div2); /* Set GPT1 source to SYSTEM PLL1 DIV2 400MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootGpt1, 1U, 4U);                  /* Set root clock to 400MHZ / 4 = 100MHZ */

    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootGpt1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootGpt1)) / 2     /* SYSTEM PLL1 DIV2 */;

    uint32_t ticks = freq / 1000000UL * period_us / divider;
    printf("GPT freq = %u\r\n", freq);
    printf("%u ticks for %uus\r\n", ticks, period_us);

    gpt_config_t gptConfig;
    GPT_GetDefaultConfig(&gptConfig);
    GPT_Init(GPT1, &gptConfig);
    GPT_SetClockDivider(GPT1, divider);
    GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, ticks);
    GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);
    EnableIRQ(GPT1_IRQn);
    GPT_StartTimer(GPT1);

    for(;;);
}
