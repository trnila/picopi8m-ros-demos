cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(uart4 uart4.c) 
target_link_libraries(uart4 mcuxpressosdk)

add_firmware(uart3 uart3.c) 
target_link_libraries(uart3 mcuxpressosdk)

