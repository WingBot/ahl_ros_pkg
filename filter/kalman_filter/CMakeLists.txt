cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

#################################################################################
# Find dependencies
#################################################################################

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    tf2
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

#################################################################################
# Define package
#################################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    kalman_filter
  CATKIN_DEPENDS
    roscpp
    tf2
  DEPENDS
    Eigen
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

#################################################################################
# Build
#################################################################################

add_library(
  kalman_filter
    src/kalman_filter.cpp
)

target_link_libraries(
  kalman_filter
    ${catkin_LIBRARIES}
)

add_executable(
  kf_test
     test/kf_test.cpp
)

target_link_libraries(
  kf_test
    kalman_filter
)

#################################################################################
# Install
#################################################################################

install(
  TARGETS
    kalman_filter
    kf_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)