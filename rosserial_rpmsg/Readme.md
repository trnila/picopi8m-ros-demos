# rosserial_rpmsg
Rosserial_rpmsg uses rpmsg_lite for communication from m4 core to linux.

At first, generate library for m4 core with:
```sh
$ rosrun rosserial_rpmsg make_libraries.py .
```

Then start rosserial serial_node.py:
```sh
$ rosrun rosserial_python serial_node.py /dev/ttyRPMSG30
```
This requires loaded imx_rpmsg_tty driver.
