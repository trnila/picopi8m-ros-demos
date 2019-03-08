cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(adxl345_rtos i2c2_adxl345.cpp) 
target_link_libraries(adxl345_rtos mcuxpressosdk freertos)
