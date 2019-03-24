cmake_minimum_required(VERSION 3.0.0)

project(gpio_interrupts)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(gpio_interrupts main.cpp interrupts.cpp) 
target_link_libraries(gpio_interrupts rosserial_rpmsg)
