cmake_minimum_required(VERSION 3.0.0)

project(hello)
find_package(M4SDK REQUIRED)

include_directories(.)

add_firmware(hello hello.c) 
target_link_libraries(hello mcuxpressosdk)

