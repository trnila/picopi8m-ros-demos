cmake_minimum_required(VERSION 3.0.0)

project(coredump)
find_package(M4SDK REQUIRED)

add_firmware(hello hello.c) 
target_link_libraries(hello mcuxpressosdk)
