cmake_minimum_required(VERSION 3.0.0)

project(spi_adxl345)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)

include_directories(.)

add_firmware(adxl345_rtos adxl345_rtos.c) 
target_link_libraries(adxl345_rtos mcuxpressosdk freertos)
