# gpio_interrupts
Example counts falling/rising edges on M4 core and sends them to the ROS.

Node serial_node.py must be running.

To start M4 code and linux example:

```sh
$ (cd m4; m4run)
$ rosrun gpio_interrupts show
```

## enable GPIO interrupts via ROS service
```sh
$ rosservice call gpio_interrupts/enable 4 23 1
```

## get current count of interrupts
```sh
$ rostopic echo /gpio_interrupts/counts
counts:
  -
    port: 3
    pin: 3
    count: 0
  -
    port: 4
    pin: 23
    count: 1
  -
    port: 4
    pin: 26
    count: 1075
---
```
