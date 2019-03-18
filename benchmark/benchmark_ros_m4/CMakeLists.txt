cmake_minimum_required(VERSION 2.8.3)
project(benchmark_ros_m4)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

# automatically build m4 application
find_program(M4BUILD m4build)
IF(M4BUILD)
  add_custom_target(benchmark_ros_m4_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)
ENDIF()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark ${catkin_LIBRARIES})
