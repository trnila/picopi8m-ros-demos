#!/bin/bash
while true; do
  for i in {0..100}; do
    cat <<EOF
servoA: $((500 + 2000 * i/100))
servoB: $((500 + 2000 * (100-i)/100))
motorA: $i
motorB: $((100 - i))
---
EOF
  done
done | rostopic pub -r 50 /motors carmotor_msgs/CarMotor
