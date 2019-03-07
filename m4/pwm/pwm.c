#include <stdio.h>
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_pwm.h"
#include "pin_mux.h"
#include "clock_config.h"

void BOARD_InitPins(void) {
    // PWM2
    IOMUXC_SetPinMux(IOMUXC_SPDIF_RX_PWM2_OUT, 0U);
    
    // PWM3
    IOMUXC_SetPinMux(IOMUXC_I2C3_SDA_PWM3_OUT, 0U); // GPIO5_IO19 (147)
    IOMUXC_SetPinMux(IOMUXC_SPDIF_TX_PWM3_OUT, 0U); // GPIO5_IO3  (131)

    // PWM4
    IOMUXC_SetPinMux(IOMUXC_I2C3_SCL_PWM4_OUT, 0U); // GPIO5_IO18 (146)
    IOMUXC_SetPinMux(IOMUXC_SAI3_MCLK_PWM4_OUT, 0U); // GPIO5_IO2 (130)
}

uint64_t get_clock_frequency(pwm_clock_source_t src) {
  switch(src) {
    case kPWM_PeripheralClock:
      return 25000000;

    case kPWM_LowFrequencyClock:
      return 32768;
  }

  return 0;
}


void pwm(PWM_Type *base, pwm_clock_source_t clksrc, int t_us, int duty_percent) {
    pwm_config_t pwmConfig;
    PWM_GetDefaultConfig(&pwmConfig);
    pwmConfig.clockSource = clksrc;
    PWM_Init(base, &pwmConfig);


    int period = t_us * get_clock_frequency(clksrc) / 1000000UL /* 1 sec */ - 2;
    int duty = duty_percent * period / 100;

    PWM_SetSampleValue(base, duty);
    PWM_SetPeriodValue(base, period);
    PWM_StartTimer(base);
}

// you should see pwm output on all PWM pins specified on pinmux
int main(void) {
    BOARD_RdcInit();
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    BOARD_InitMemory();

    printf("PWM example started\r\n");

    // 32 768 Hz low frequency clock, period 10ms, duty 1ms
    // pins GPIO5_IO4 (132)
    pwm(PWM2, kPWM_LowFrequencyClock, 10000, 10);

    // 25 Mhz peripheral clock, period 1 ms, duty 0.1ms
    // pins GPIO5_IO19 (147), GPIO5_IO3  (131)
    pwm(PWM3, kPWM_PeripheralClock, 1000, 10);

    // TODO: add example for high frequency
    // 25 Mhz peripheral clock, period 0.5 ms, duty 0.05ms
    // pins GPIO5_IO2 (130), GPIO5_IO18 (146)
     pwm(PWM4, kPWM_PeripheralClock, 500, 10);
    for(;;);
}
