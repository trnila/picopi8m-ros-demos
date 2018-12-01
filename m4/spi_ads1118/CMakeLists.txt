cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(ads1118_busy_waiting ads1118_busy_waiting.c) 
target_link_libraries(ads1118_busy_waiting mcuxpressosdk)

add_firmware(ads1118_irq ads1118_irq.c) 
target_link_libraries(ads1118_irq mcuxpressosdk)
