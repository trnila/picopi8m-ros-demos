cmake_minimum_required(VERSION 3.0.0)

project(print)
find_package(M4SDK REQUIRED)
find_package(freertos REQUIRED)

add_firmware(print print.c) 
target_link_libraries(print freertos mcuxpressosdk)

