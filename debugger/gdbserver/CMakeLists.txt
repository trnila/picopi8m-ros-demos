cmake_minimum_required(VERSION 3.0.0)

project(debugger)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

add_firmware(div_zero div_zero.c debug.c) 
target_link_libraries(div_zero mcuxpressosdk)

add_firmware(callstack callstack.c debug.c) 
target_link_libraries(callstack mcuxpressosdk)
