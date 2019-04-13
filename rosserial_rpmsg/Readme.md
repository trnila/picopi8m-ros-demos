# rosserial_rpmsg
Rosserial_rpmsg uses rpmsg_lite for communication from m4 core to linux.

M4 code requires rosserial client library and generated header files for messages and services.
There files are automatically generated with CMake library/package **rosserial_rpmsg**.
You just have to load that library and link with your application:
```cmake
find_package(rosserial_rpmsg REQUIRED)

add_firmware(hello_m4char hello.cpp) 
target_link_libraries(hello_m4char rosserial_rpmsg)
```

You can create these files manually with:
```sh
$ rosrun rosserial_rpmsg make_libraries.py .
```

Node **serial_node.py** must be running for passing messages/service calls from ROS to M4 core and vice versa.
```sh
$ rosrun rosserial_rpmsg serial_node.py /dev/m4char
```

Or you can use imx_rpmsg_tty driver, but you have to probe it first:
```sh
$ modprobe imx_rpmsg_tty
$ rosrun rosserial_rpmsg serial_node.py /dev/ttyRPMSG30
```

Example use can be found in **example** or **../ros_m4_demo* directory.
