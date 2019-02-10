# linecam
Example shows how to measure data from analog linecam that is connected to picopi via SPI ADC128S102.
Linecam prepares first pixel of 128 on analog output when SI signal goes up with CLK clock.
That value is read from ADC.
Next pixel is prepared on following CLK pulse.
Whole row is then transmitted over rpmsg to the *rosserial_rpmsg/serial_node.py* node, which will deliver row to all subscribers of */linecam0* topic for further processing.

At first compile your ros workspace with `catkin_make` which is in *~/catkin_ws* by default.
Firmware for M4 core will be also built in this step.

Then you can load firmware, start *rosserial_rpmsg/serial_node.py* and *rosbridge* with single command:
```sh
$ roslaunch linecam app.launch
```
One of example subscribers is *display* showing measured data in shifting image.
```sh
$ rosrun linecam display
```
![display example](display.jpg)


Another example may be `rosbridge_websocket`, that can bridge ros functionality with websocket, so it's possible to visualize measured data directly on web page as in *display.html* example.
Open it in your browser and optionally set some settings via URL fragment - *#server=picopi&zoom=1&topic=/linecam0*.

## Ros recordings
Data on ros topics can be recorded and used for later debugging.
To recored data just run:
```sh
$ rosbag record --duration 5 -O captures/ros.bag /linecam0
```

Replay data with:
```sh
$ rosbag play captures/ros.bag
```
