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
#include "motors.h"

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
