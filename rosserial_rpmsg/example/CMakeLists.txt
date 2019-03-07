cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(hello hello.cpp ros_lib/time.cpp)
target_link_libraries(hello mcuxpressosdk freertos rpmsg_lite)

add_firmware(fake_sensor_m4char fake_sensor.cpp ros_lib/time.cpp) 
target_link_libraries(fake_sensor_m4char mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(fake_sensor_m4char PRIVATE RPMSG_CHANNEL="m4-channel") 

add_firmware(fake_sensor_tty fake_sensor.cpp ros_lib/time.cpp) 
target_link_libraries(fake_sensor_tty mcuxpressosdk freertos rpmsg_lite)
target_compile_definitions(fake_sensor_tty PRIVATE RPMSG_CHANNEL="rpmsg-virtual-tty-channel-1") 
