cmake_minimum_required(VERSION 3.0.0)

project(benchmark_ros_m4)
find_package(M4SDK REQUIRED)
find_package(FreeRTOS REQUIRED)
find_package(rpmsg REQUIRED)

include_directories(.)
include_directories(ros_lib)

add_firmware(bench_rostopic main.cpp ros_lib/time.cpp) 
target_link_libraries(bench_rostopic mcuxpressosdk freertos rpmsg_lite)
