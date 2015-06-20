cmake_minimum_required(VERSION 2.8.3)
project(ahl_lwr_server)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
)

catkin_package(
  CATKIN_DEPENDS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  lwr_server_test
    test/test.cpp
)

target_link_libraries(
  lwr_server_test
    ${catkin_LIBRARIES}
)

