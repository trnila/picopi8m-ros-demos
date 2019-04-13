cmake_minimum_required(VERSION 2.8.3)
project(ros_m4ctl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
   FILES
   Start.srv
   Stop.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_m4ctl src/m4ctrl.cpp)
target_link_libraries(ros_m4ctl ${catkin_LIBRARIES})
add_dependencies(ros_m4ctl ros_m4ctl_generate_messages_cpp)

#add_executable(hello src/hello.cpp)
#target_link_libraries(hello ${catkin_LIBRARIES})
#add_dependencies(hello ros_m4ctl_generate_messages_cpp)

# automatically build m4 application
find_program(M4BUILD m4build)
IF(M4BUILD)
  add_custom_target(ros_m4ctl_m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)
ENDIF()
