cmake_minimum_required(VERSION 2.8.3)
project(manipit)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_LIBRARY_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    manipit
  CATKIN_DEPENDS
    cv_bridge
    geometry_msgs
    image_transport
    message_filters
    roscpp
    sensor_msgs
    std_srvs
    tf2
    tf2_ros
  DEPENDS
    Eigen
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

##############################################################################
# Build
##############################################################################

add_library(
  manipit
    src/manipit_server.cpp
    src/manipit_server_impl.cpp
    src/manipit.cpp
    src/hand_recognizer/deep_learning_depth.cpp
    src/hand_detector/rgb_hand_detector.cpp
)

target_link_libraries(
  manipit
    ${catkin_LIBRARIES}
)

add_executable(
  manipit_node
    src/main.cpp
)

target_link_libraries(
  manipit_node
    manipit
)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    manipit
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)