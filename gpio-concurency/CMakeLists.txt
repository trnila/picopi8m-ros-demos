cmake_minimum_required(VERSION 3.0.0)

project(gpio_concurency)
find_package(M4SDK REQUIRED)

include_directories(.)

add_firmware(m4 m4.c) 
target_link_libraries(m4 mcuxpressosdk)

