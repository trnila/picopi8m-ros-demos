cmake_minimum_required(VERSION 2.8.3)
project(m4_hello_tty)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(proxy src/proxy.cpp src/m4core.cpp)
target_link_libraries(proxy ${catkin_LIBRARIES})

add_executable(send src/send.cpp src/m4core.cpp)
target_link_libraries(send ${catkin_LIBRARIES})

add_executable(generator src/generator.cpp src/m4core.cpp)
target_link_libraries(generator ${catkin_LIBRARIES})
