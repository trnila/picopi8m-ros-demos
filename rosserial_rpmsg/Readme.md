# rosserial_rpmsg
Rosserial_rpmsg uses rpmsg_lite for communication from m4 core to linux.

At first, generate library for m4 core at your project with:
```sh
$ rosrun rosserial_rpmsg make_libraries.py .
```

Then start rosrpmsg serial_node.py:
```sh
$ rosrun rosserial_rpmsg serial_node.py /dev/ttyRPMSG30
```
This requires loaded imx_rpmsg_tty driver.

Or you can use m4char:
```sh
$ rosrun rosserial_rpmsg serial_node.py /dev/m4char
```

