#!/bin/bash
while true; do
  for i in {500..2500..50}; do
    cat <<EOF
servoA: $i
servoB: 2000
motorA: 60
motorB: 20
---
EOF
  done
done | rostopic pub -r 50 /motors carmotor_msgs/CarMotor
