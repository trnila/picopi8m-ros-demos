cmake_minimum_required(VERSION 3.0.0)

project(rpmsg)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(return_borrowed return_borrowed.c) 
target_link_libraries(return_borrowed mcuxpressosdk)
