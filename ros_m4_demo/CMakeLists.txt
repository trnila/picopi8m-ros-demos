cmake_minimum_required(VERSION 2.8.3)
project(ros_m4_demo)

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  message_generation
  genmsg
)

add_message_files(FILES Location.msg)
add_message_files(FILES Print.msg)
add_service_files(FILES Pause.srv)
add_service_files(FILES Addition.srv)
add_service_files(FILES SetSigma.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
)

# automatically build m4 application
#add_custom_target(m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber ros_m4_demo_generate_messages_cpp)

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
add_dependencies(publisher ros_m4_demo_generate_messages_cpp)

add_executable(service_call src/service_call.cpp)
target_link_libraries(service_call ${catkin_LIBRARIES})
add_dependencies(service_call ros_m4_demo_generate_messages_cpp)

add_executable(service_server src/service_server.cpp)
target_link_libraries(service_server ${catkin_LIBRARIES})
add_dependencies(service_server ros_m4_demo_generate_messages_cpp)
