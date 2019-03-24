cmake_minimum_required(VERSION 3.0.0)

project(fake_m4_sensor)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(linecam_simple main.cpp simple.cpp) 
target_link_libraries(linecam_simple rosserial_rpmsg)

add_firmware(linecam_irq main.cpp irq.cpp) 
target_link_libraries(linecam_irq rosserial_rpmsg)

add_firmware(linecam_raw main.cpp raw.cpp) 
target_link_libraries(linecam_raw rosserial_rpmsg)


add_firmware(linecam_debug main.cpp simple.cpp) 
target_link_libraries(linecam_debug rosserial_rpmsg)
target_compile_definitions(linecam_debug PRIVATE PRINT_OUTPUT=1) 
