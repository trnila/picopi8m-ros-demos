cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(gpio_blink gpio_blink.c) 
target_link_libraries(gpio_blink mcuxpressosdk)

add_firmware(gpio_input gpio_input.c) 
target_link_libraries(gpio_input mcuxpressosdk)

add_firmware(gpio_irq gpio_irq.c) 
target_link_libraries(gpio_irq mcuxpressosdk)

add_firmware(gpio_flip gpio_flip.c) 
target_link_libraries(gpio_flip mcuxpressosdk)
