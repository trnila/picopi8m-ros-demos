cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(hello hello.cpp pin_mux.c ros_lib/time.cpp) 
target_link_libraries(hello mcuxpressosdk freertos rpmsg_lite)

add_firmware(fake_sensor fake_sensor.cpp pin_mux.c ros_lib/time.cpp) 
target_link_libraries(fake_sensor mcuxpressosdk freertos rpmsg_lite)
