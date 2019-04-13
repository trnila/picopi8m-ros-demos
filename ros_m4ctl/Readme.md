# ros_m4ctl
ROS package mainly for loading M4 code from roslaunch.

Add node **ros_m4ctl/m4ctl** to your roslaunch configuration, so M4 application will start with other nodes:
```xml
<launch>
  <node name="m4_loader" pkg="ros_m4ctl" type="m4ctl" args="start $(find ros_m4ctl)/m4/build/debug/hello" />
</launch>
```

To build M4 code as a part of ROS build process, add following snippet to ROS CMake configuration:
```cmake
# automatically build m4 application
find_program(M4BUILD m4build)
IF(M4BUILD)
  add_custom_target(ros_m4ctl_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)
ENDIF()  
```
