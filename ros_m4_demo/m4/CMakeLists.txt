cmake_minimum_required(VERSION 3.0.0)

project(ros_m4_demo)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(ros_m4_demo main.cpp ros_lib/time.cpp) 
target_link_libraries(ros_m4_demo mcuxpressosdk freertos rpmsg_lite)
