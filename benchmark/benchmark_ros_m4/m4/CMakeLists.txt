cmake_minimum_required(VERSION 3.0.0)

project(benchmark_ros_m4)
find_package(rosserial_rpmsg REQUIRED)

add_firmware(bench_rostopic main.cpp) 
target_link_libraries(bench_rostopic rosserial_rpmsg)
