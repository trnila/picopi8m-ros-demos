cmake_minimum_required(VERSION 3.0.0)

project(ros_m4_demo)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(ros_m4_demo main.cpp) 
target_link_libraries(ros_m4_demo rosserial_rpmsg)
