cmake_minimum_required(VERSION 2.8.3)
project(carmotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  genmsg
)

add_message_files(FILES CarMotor.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS
  std_msgs
  message_runtime
)

# automatically build m4 application
#add_custom_target(m4 ALL m4build WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/m4)
