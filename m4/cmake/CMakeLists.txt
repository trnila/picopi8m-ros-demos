cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(bare bare.c) 
target_link_libraries(bare mcuxpressosdk)

add_firmware(rtos rtos.c) 
target_link_libraries(rtos mcuxpressosdk freertos)

add_firmware(rpmsg rpmsg.c) 
target_link_libraries(rpmsg mcuxpressosdk freertos rpmsg_lite)
