cmake_minimum_required(VERSION 2.8.3)
project(ahl_gazebo_interface)

find_package(
  catkin REQUIRED COMPONENTS
    gazebo_msgs
    roscpp
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_gazebo_interface
  CATKIN_DEPENDS
    gazebo_msgs
    roscpp
  DEPENDS
    Eigen
)

add_library(
  ahl_gazebo_interface
    src/gazebo_interface.cpp
)

add_dependencies(
  ahl_gazebo_interface
    gazebo_msgs_gencpp
)

target_link_libraries(
  ahl_gazebo_interface
    ${catkin_LIBRARIES}
)
