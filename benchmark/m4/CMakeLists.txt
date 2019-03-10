cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(ping_m4char ping.c) 
target_link_libraries(ping_m4char mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(ping_m4char PRIVATE RPMSG_CHANNEL="m4-channel") 

add_firmware(ping_tty ping.c) 
target_link_libraries(ping_tty mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(ping_tty PRIVATE RPMSG_CHANNEL="rpmsg-virtual-tty-channel-1") 

add_firmware(mu mu.c) 
target_link_libraries(mu mcuxpressosdk freertos rpmsg_lite)
