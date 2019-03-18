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
find_program(M4BUILD m4build)
IF(M4BUILD)
  add_custom_target(gpio_interrupts_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)
ENDIF()


include_directories(${catkin_INCLUDE_DIRS})

add_executable(show src/show.cpp)
target_link_libraries(show ${catkin_LIBRARIES})
add_dependencies(show gpio_interrupts_generate_messages_cpp)
