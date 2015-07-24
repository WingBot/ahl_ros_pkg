cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_samples)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    gazebo_msgs
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    ahl_gazebo_interface
    ahl_robot
    ahl_robot_controller
    gazebo_msgs
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  pr2_sample
    src/pr2/main.cpp
    src/pr2/pr2.cpp
)

add_dependencies(
  pr2_sample
    gazebo_msgs_gencpp
)

target_link_libraries(
  pr2_sample
    ${catkin_LIBRARIES}
)
