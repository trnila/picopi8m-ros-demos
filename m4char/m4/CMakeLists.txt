cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(strupper main.c) 
target_link_libraries(strupper mcuxpressosdk freertos rpmsg_lite)
