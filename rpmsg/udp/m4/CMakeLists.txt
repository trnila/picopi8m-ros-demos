cmake_minimum_required(VERSION 3.0.0)

project(rpmsg_udp)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(rpmsg_udp rpmsg_udp.c) 
target_link_libraries(rpmsg_udp mcuxpressosdk freertos rpmsg_lite)
