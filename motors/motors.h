#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// standard range 500-2500 usec
void servo_start();
void servoA_set(uint16_t usec);
void servoB_set(uint16_t usec);

// 0-100 %
void motor_start();
void motorA_set(uint8_t value);
void motorB_set(uint8_t value);

#ifdef __cplusplus
}
#endif
