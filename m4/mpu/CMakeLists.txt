cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)

add_firmware(mpu_disable_ddr mpu_disable_ddr.c) 
target_link_libraries(mpu_disable_ddr mcuxpressosdk)

add_firmware(mpu_freertos mpu_freertos.c) 
target_link_libraries(mpu_freertos mcuxpressosdk freertos)

add_firmware(mpu_rpmsg mpu_rpmsg.c) 
target_link_libraries(mpu_rpmsg mcuxpressosdk freertos rpmsg_lite)
