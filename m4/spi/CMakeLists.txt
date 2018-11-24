cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(spi1 spi1.c) 
target_link_libraries(spi1 mcuxpressosdk)

add_firmware(spi2 spi2.c) 
target_link_libraries(spi2 mcuxpressosdk)

add_firmware(spi3 spi3.c) 
target_link_libraries(spi3 mcuxpressosdk)

add_firmware(spi1_hw_cs spi1_hw_cs.c) 
target_link_libraries(spi1_hw_cs mcuxpressosdk)

add_firmware(spi1_gpio_cs spi1_gpio_cs.c) 
target_link_libraries(spi1_gpio_cs mcuxpressosdk)
