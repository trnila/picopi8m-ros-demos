cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(rdc_gpio3_m4_only rdc_gpio3_m4_only.c) 
target_link_libraries(rdc_gpio3_m4_only mcuxpressosdk)
