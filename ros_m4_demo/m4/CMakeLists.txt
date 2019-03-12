cmake_minimum_required(VERSION 3.0.0)

project(fake_m4_sensor)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(fake_m4_sensor main.cpp ros_lib/time.cpp) 
target_link_libraries(fake_m4_sensor mcuxpressosdk freertos rpmsg_lite)
