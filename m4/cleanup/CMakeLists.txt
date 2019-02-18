cmake_minimum_required(VERSION 3.0.0)

project(cleanup)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(bare bare.c) 
target_link_libraries(bare mcuxpressosdk)
