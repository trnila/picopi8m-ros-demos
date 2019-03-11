cmake_minimum_required(VERSION 3.0.0)

project(motors)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(motors_demo demo.c motors.c) 
target_link_libraries(motors_demo mcuxpressosdk)

add_firmware(motors_ros motors.c ros.cpp ros_lib/time.cpp) 
target_link_libraries(motors_ros mcuxpressosdk freertos rpmsg_lite)
