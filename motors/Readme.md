# example for servo & dc motor control
- PWM3/4 is used for servos
  - SERVO_A - PWM3 routed to pins GPIO5_IO3 and GPIO5_19
  - SERVO_B - PWM4 routed to pins GPIO5_IO2 AND GPIO5_18
- GPT1 for toggling duty cycle for dc motors
  - MOTOR_A - GPIO3_IO15
  - MOTOR_B - GPIO3_IO17

![capture](capture.png) 

## start
```sh
$ m4run motors_ros
```

Node **rosserial_rpmsg/serial_node.py** should be running!
```sh
$ rosrun rosserial_rpmsg serial_node.py /dev/m4char
```

## set value from linux
```sh
$ ./feed.sh
$ rostopic pub -1 /servo/A std_msgs/UInt16 1500 # usec
$ rostopic pub -1 /servo/B std_msgs/UInt16 500  # usec
$ rostopic pub -1 /motor/A std_msgs/Byte 25     # %
$ rostopic pub -1 /motor/B std_msgs/Byte 75     # %
```
