cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(systick systick.c) 
target_link_libraries(systick mcuxpressosdk)

add_firmware(gpt1 gpt1.c) 
target_link_libraries(gpt1 mcuxpressosdk)
