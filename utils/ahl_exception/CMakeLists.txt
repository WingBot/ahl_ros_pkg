cmake_minimum_required(VERSION 2.8.3)
project(ahl_exception)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
