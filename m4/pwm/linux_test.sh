#!/bin/sh
# PWM2, PWM3, PWM4 is available if you load pico-8m-pwm.dtb

num=$1

if [ -z "$num" ]; then
  echo Usage: $(basename "$0") pwm_num
  exit 1
fi

cd "/sys/class/pwm/pwmchip$num"

echo 0 > export
echo 1000000000 > pwm0/period
echo 100000 > pwm0/duty_cycle
echo 1 > pwm0/enable
