cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(m4char rpmsg.c pin_mux.c) 
target_link_libraries(m4char mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(m4char PRIVATE RPMSG_CHANNEL="m4-channel") 

add_firmware(tty rpmsg.c pin_mux.c) 
target_link_libraries(tty mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(tty PRIVATE RPMSG_CHANNEL="rpmsg-virtual-tty-channel-1") 
