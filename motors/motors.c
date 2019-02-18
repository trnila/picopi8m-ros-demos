#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "benchmark.h"

#define PORT_MOTORA GPIO3
#define PORT_MOTORB GPIO3
#define PIN_MOTORA 15
#define PIN_MOTORB 17

#define PWM_SERVOA PWM3
#define PWM_SERVOB PWM4

uint32_t servo_freq;
uint32_t motor_ticks;

void BOARD_InitPins(void) {
	// we are routing PWM functionality to both pins
	// PWM3
	IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_PWM3_OUT, 0U); // GPIO5_IO19 (147)
	IOMUXC_SetPinMux(IOMUXC_SPDIF_TX_PWM3_OUT, 0U); // GPIO5_IO3  (131)

	// PWM4
	IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_PWM4_OUT, 0U); // GPIO5_IO18 (146)
	IOMUXC_SetPinMux(IOMUXC_SAI3_MCLK_PWM4_OUT, 0U); // GPIO5_IO2 (130)

	IOMUXC_SetPinMux(IOMUXC_NAND_RE_B_GPIO3_IO15, 0U);
	IOMUXC_SetPinConfig(IOMUXC_NAND_RE_B_GPIO3_IO15, 
			IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
			IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
	IOMUXC_SetPinMux(IOMUXC_NAND_WE_B_GPIO3_IO17, 0U);
	IOMUXC_SetPinConfig(IOMUXC_NAND_WE_B_GPIO3_IO17, 
			IOMUXC_SW_PAD_CTL_PAD_DSE(6U) |
			IOMUXC_SW_PAD_CTL_PAD_SRE(3U));
}


void servo_start() {
	uint16_t prescaler = 7 /* -1 */;
	uint32_t period_ms = 20;

	uint32_t clock = 25000000U;
	servo_freq = clock / (prescaler + 1);

	uint32_t period_ticks = period_ms * servo_freq / 1000 /* ms */;
	assert(period_ticks < 0xFFFF);

	pwm_config_t pwmConfig;
	PWM_GetDefaultConfig(&pwmConfig);
	pwmConfig.clockSource = kPWM_PeripheralClock;
	pwmConfig.prescale = prescaler;
	PWM_Init(PWM_SERVOA, &pwmConfig);
	PWM_Init(PWM_SERVOB, &pwmConfig);

	PWM_SetPeriodValue(PWM_SERVOA, period_ticks);
	PWM_SetPeriodValue(PWM_SERVOB, period_ticks);

	PWM_StartTimer(PWM_SERVOA);
	PWM_StartTimer(PWM_SERVOB);
}

void servo_set(PWM_Type *pwm, uint16_t usec) {
	uint16_t ticks = (uint64_t) usec * servo_freq / 1000000;
	PWM_SetSampleValue(pwm, ticks);
}

void servoA_set(uint16_t usec) {
	servo_set(PWM_SERVOA, usec);
}

void servoB_set(uint16_t usec) {
	servo_set(PWM_SERVOB, usec);
}

void GPT1_IRQHandler() {
	if(GPT_GetStatusFlags(GPT1, kGPT_OutputCompare1Flag)) {
		GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);
		PORT_MOTORA->DR |= 1 << PIN_MOTORA;
		PORT_MOTORB->DR |= 1 << PIN_MOTORB;
	}

	if(GPT_GetStatusFlags(GPT1, kGPT_OutputCompare2Flag)) {
		GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare2Flag);
		PORT_MOTORA->DR &= ~(1 << PIN_MOTORA);
	}

	if(GPT_GetStatusFlags(GPT1, kGPT_OutputCompare3Flag)) {
		GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare3Flag);
		PORT_MOTORB->DR &= ~(1 << PIN_MOTORB);
	}
}

void motor_set(gpt_output_compare_channel_t channel, uint8_t value) {
	assert(value >= 0 && value <= 100);
	GPT_SetOutputCompareValue(GPT1, channel, value * motor_ticks / 100 /* % */);
}

void motorA_set(uint8_t value) {
	motor_set(kGPT_OutputCompare_Channel2, value);
}

void motorB_set(uint8_t value) {
	motor_set(kGPT_OutputCompare_Channel3, value);
}

void motor_start() {
	gpio_pin_config_t conf;
	conf.direction = kGPIO_DigitalOutput;
	conf.outputLogic = 0;
	conf.interruptMode = kGPIO_NoIntmode;
	GPIO_PinInit(PORT_MOTORA, PIN_MOTORA, &conf);
	GPIO_PinInit(PORT_MOTORB, PIN_MOTORB, &conf);

	CLOCK_SetRootMux(kCLOCK_RootGpt1, kCLOCK_GptRootmuxSysPll1Div20); /* Set GPT1 source to SYSTEM PLL1 DIV2 400MHZ */
	CLOCK_SetRootDivider(kCLOCK_RootGpt1, 1U, 1U);                  /* Set root clock to 400MHZ / 4 = 100MHZ */

	// 40 Mhz
	uint32_t freq = CLOCK_GetPllFreq(kCLOCK_SystemPll1Ctrl) / (CLOCK_GetRootPreDivider(kCLOCK_RootGpt1)) / (CLOCK_GetRootPostDivider(kCLOCK_RootGpt1)) / 20     /* SYSTEM PLL1 DIV20 */;

	motor_ticks = freq / 4000 /* khz */;

	gpt_config_t gptConfig;
	GPT_GetDefaultConfig(&gptConfig);
	gptConfig.enableFreeRun = false; // restart when Chnnel1 reaches output compare value of 4 khz
	GPT_Init(GPT1, &gptConfig);
	GPT_SetClockDivider(GPT1, 1);
	GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, motor_ticks);
	GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel2, motor_ticks);
	GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel3, motor_ticks);
	GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable | kGPT_OutputCompare2InterruptEnable | kGPT_OutputCompare3InterruptEnable);
	EnableIRQ(GPT1_IRQn);
	GPT_StartTimer(GPT1);
}

void delay_10ms() {
	for(int i = 0; i < 1000000 * 2 / 3; i++) asm("nop");
}

int main(void) {
	BOARD_RdcInit();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
	BOARD_InitMemory();

	// uncomment to benchmark before GPT interrupts
	// benchmark(); for(;;); /* benchmark is infinity loop! */

	servo_start();
	motor_start();

	servoA_set(500);
	servoB_set(2500);

	motorA_set(25);
	motorB_set(75);

	// uncomment to benchmark with GPT interrupts enabled
	// benchmark(); for(;;); /* benchmark is infinity loop! */

	for(;;) {
		int Max = 2000;
		for(int i = 0; i <= Max; i += 10) {
			servoA_set(500 + i);
			servoB_set(2500 - i);

			motorA_set(100 * i / Max);
			motorB_set(100 - 100 * i / Max);

			delay_10ms();
			delay_10ms();
		}
	}
}
