# ros_m4_demo - Example for ROS communication with M4 core
![ros_m4_demo](ros_m4_demo.svg)

Start all nodes with:
```sh
$ roslaunch app.launch
```
Or just load firmware and start **serial_node**:
```sh
$ roslaunch base.launch
```

## Receive messages from M4 core
```sh
$ rostopic echo /ros_m4_demo/hello
data: "Hello 2579!"
---
data: "Hello 2580!"
---
```

```sh
$ rosrun ros_m4_demo subscriber
Hello 2684!
-1.238578 5.378663 12.401332
16.218216 17.456795 12.078133
Hello 2685!
-13.978265 -30.196480 -47.653275
2.672040 16.650305 46.846786
```

Filter data from M4 core with Kalman Filter and publish them to another topic:
```sh
$ rosrun ros_m4_demo kalman_filter.py
[INFO] [1555158861.905520]: Filtered value:
position: [ 1.52996377]
velocity: [ 1.02008917]
acceleration: [ 0.34014305]
real_position: 0.0
[INFO] [1555158862.005119]: Filtered value:
position: [ 0.16121998]
velocity: [-0.38298326]
acceleration: [-0.25033636]
real_position: 0.0

```

## Send message to M4 core
```sh
$ rostopic pub -1 /ros_m4_demo/print ros_m4_demo/Print "Hello World" 64
```

```sh
$ rosrun ros_m4_demo publisher
```

## Call service on the M4 core
```sh
$ rosservice call ros_m4_demo/pause 1
count: 2
$ rosservice call ros_m4_demo/set_sigma 7.5
```

## Provide service on the Linux side
```sh
$ rosrun ros_m4_demo service_server
Calculating 5 + 150
Calculating 6 + 160
Calculating 7 + 170
```
Currently, service MUST start before M4 core.
