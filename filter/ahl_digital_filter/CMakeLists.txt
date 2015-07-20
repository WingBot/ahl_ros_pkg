cmake_minimum_required(VERSION 2.8.3)
project(ahl_digital_filter)

find_package(
  catkin REQUIRED COMPONENTS
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
    ahl_digital_filter
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    Eigen
)

add_library(
  ahl_digital_filter
    src/pseudo_differentiator.cpp
)

target_link_libraries(
  ahl_digital_filter
    ${catkin_LIBRARIES}
)

add_executable(
  ahl_digital_filter_test
    test/test.cpp
)

target_link_libraries(
  ahl_digital_filter_test
    ahl_digital_filter
)
