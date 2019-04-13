cmake_minimum_required(VERSION 3.0.0)

project(rosserial_rpmsg_example)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(hello_m4char hello.cpp) 
target_link_libraries(hello_m4char rosserial_rpmsg)
target_compile_definitions(hello_m4char PRIVATE RPMSG_CHANNEL="m4-channel") 

add_firmware(hello_tty hello.cpp) 
target_link_libraries(hello_tty rosserial_rpmsg)
target_compile_definitions(hello_tty PRIVATE RPMSG_CHANNEL="rpmsg-virtual-tty-channel-1") 
