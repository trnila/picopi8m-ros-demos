cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(i2c2 i2c2.c ds1621) 
target_link_libraries(i2c2 mcuxpressosdk)

add_firmware(i2c3 i2c3.c ds1621) 
target_link_libraries(i2c3 mcuxpressosdk)
