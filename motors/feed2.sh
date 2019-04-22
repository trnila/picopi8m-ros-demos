#!/bin/bash
while true; do
  for i in {500..2500..50}; do
    cat <<EOF
$i
---
EOF
  done
done | rostopic pub -r 50 /servo/A std_msgs/UInt16
