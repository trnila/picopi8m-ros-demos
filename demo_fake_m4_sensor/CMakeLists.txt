cmake_minimum_required(VERSION 2.8.3)
project(demo_fake_m4_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  message_generation
  genmsg
)

add_message_files(FILES Location.msg)
add_service_files(FILES Pause.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

# automatically build m4 application
add_custom_target(m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(bridge src/bridge.cpp)
target_link_libraries(bridge ${catkin_LIBRARIES})
add_dependencies(bridge demo_fake_m4_sensor_generate_messages_cpp)
