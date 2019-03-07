cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(adc128s102_simple adc128s102_simple.c) 
target_link_libraries(adc128s102_simple mcuxpressosdk)

add_firmware(adc128s102_single adc128s102_single.c) 
target_link_libraries(adc128s102_single mcuxpressosdk)

add_firmware(adc128s102_rtos adc128s102_rtos.c) 
target_link_libraries(adc128s102_rtos mcuxpressosdk freertos)

add_firmware(adc128s102_sw adc128s102_sw.c) 
target_link_libraries(adc128s102_sw mcuxpressosdk freertos)

add_firmware(adc128s102_raw adc128s102_raw.c) 
target_link_libraries(adc128s102_raw mcuxpressosdk freertos)
