cmake_minimum_required(VERSION 2.8.3)
project(gpio_interrupts)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  message_generation
  genmsg
)

add_message_files(FILES Count.msg)
add_message_files(FILES Counts.msg)
add_service_files(FILES Enable.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

# automatically build m4 application
#add_custom_target(linecam_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(show src/show.cpp)
target_link_libraries(show ${catkin_LIBRARIES})
