#!/bin/sh
# PWM2, PWM3, PWM4 is available if you load pico-8m-pwm.dtb

num=$1
period_ns=$2
duty_ns=$3

if [ $# -lt 3 ]; then
  echo Usage: $(basename "$0") pwm_num period_ns duty_ns
  exit 1
fi

cd "/sys/class/pwm/pwmchip$num" || exit

if [ ! -d pwm0 ]; then
  echo 0 > export
fi

# when changing period, set duty cycle to 0
if [ "$(cat pwm0/period)" -ne "$period_ns" ]; then
  echo 0 > pwm0/duty_cycle
fi

echo $period_ns > pwm0/period 
if [ $? -ne 0 ]; then
  echo Failed to set period
fi

echo $duty_ns > pwm0/duty_cycle 
if [ $? -ne 0 ]; then
  echo Failed to set duty
fi

echo 1 > pwm0/enable
