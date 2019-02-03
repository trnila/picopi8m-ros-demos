cmake_minimum_required(VERSION 2.8.3)
project(m4ctrl)

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

#add_executable(m4ctl src/m4ctl.cpp src/m4core.cpp)
#target_link_libraries(m4ctl ${catkin_LIBRARIES})

add_executable(hello src/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES})
