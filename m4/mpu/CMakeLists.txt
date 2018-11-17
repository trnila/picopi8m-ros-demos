cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(mpu_disable_ddr mpu_disable_ddr.c) 
target_link_libraries(mpu_disable_ddr mcuxpressosdk)
