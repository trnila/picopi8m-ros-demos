cmake_minimum_required(VERSION 3.0.0)

project(fake_m4_sensor)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(linecam_simple main.cpp simple.cpp ros_lib/time.cpp) 
target_link_libraries(linecam_simple mcuxpressosdk freertos rpmsg_lite)

add_firmware(linecam_irq main.cpp irq.cpp ros_lib/time.cpp) 
target_link_libraries(linecam_irq mcuxpressosdk freertos rpmsg_lite)

add_firmware(linecam_raw main.cpp raw.cpp ros_lib/time.cpp) 
target_link_libraries(linecam_raw mcuxpressosdk freertos rpmsg_lite)
