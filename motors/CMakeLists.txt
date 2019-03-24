cmake_minimum_required(VERSION 3.0.0)

project(motors)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(motors_demo demo.c motors.c) 
target_link_libraries(motors_demo mcuxpressosdk)

add_firmware(motors_ros motors.c ros.cpp) 
target_link_libraries(motors_ros rosserial_rpmsg)
