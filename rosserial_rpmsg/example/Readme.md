# rosserial_rpmsg for communication between M4 core and ROS
Receive messages from M4 core with:
```sh
$ rostopic echo /chatter
```

## rpmsg_m4char
### Manual run
```sh
$ m4run hello_m4char 
$ rosrun rosserial_rpmsg serial_node.py /dev/m4char 
```

### with roslaunch
```sh
$ roslaunch launch_m4char.xml
```

## imx_rpmsg_tty
### Manual run
```sh
$ m4run hello_ttu 
$ modprobe imx_rpmsg_tty
$ rosrun rosserial_rpmsg serial_node.py /dev/ttyRPMSG30
```

### with roslaunch
```sh
$ roslaunch launch_tty.xml
```


