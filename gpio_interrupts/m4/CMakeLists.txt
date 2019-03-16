cmake_minimum_required(VERSION 3.0.0)

project(gpio_interrupts)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(gpio_interrupts main.cpp interrupts.cpp ros_lib/time.cpp) 
target_link_libraries(gpio_interrupts mcuxpressosdk freertos rpmsg_lite)
