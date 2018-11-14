cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(1wire_dht 1wire_dht.c) 
target_link_libraries(1wire_dht mcuxpressosdk)

add_firmware(1wire_dht_irq 1wire_dht_irq.c) 
target_link_libraries(1wire_dht_irq mcuxpressosdk)
