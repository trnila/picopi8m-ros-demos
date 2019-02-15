cmake_minimum_required(VERSION 2.8.3)
project(linecam)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
)

find_package(OpenCV REQUIRED)


catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

# automatically build m4 application
add_custom_target(linecam_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(display src/display.cpp)
target_link_libraries(display ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(linecam_linux src/linecam_linux.cpp)
target_link_libraries(linecam_linux ${catkin_LIBRARIES})

add_executable(linecam_kernel src/linecam_kernel.cpp)
target_link_libraries(linecam_kernel ${catkin_LIBRARIES})
