cmake_minimum_required(VERSION 3.0.0)

project(gpio_blink)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(hello hello.cpp)
target_link_libraries(hello rosserial_rpmsg)

add_firmware(fake_sensor_m4char fake_sensor.cpp) 
target_link_libraries(fake_sensor_m4char rosserial_rpmsg)
target_compile_definitions(fake_sensor_m4char PRIVATE RPMSG_CHANNEL="m4-channel") 

add_firmware(fake_sensor_tty fake_sensor.cpp) 
target_link_libraries(fake_sensor_tty rosserial_rpmsg)
target_compile_definitions(fake_sensor_tty PRIVATE RPMSG_CHANNEL="rpmsg-virtual-tty-channel-1") 
