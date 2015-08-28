cmake_minimum_required(VERSION 2.8.3)
project(gl_wrapper)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    cv_wrapper
    dynamic_reconfigure
    std_msgs
    std_srvs
    ahl_utils
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

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN_INCLUDE_DIRS}
  LIBRARIES
    gl_wrapper
  CATKIN_DEPENDS
    cv_bridge
    cv_wrapper
    dynamic_reconfigure
    std_msgs
    std_srvs
    ahl_utils
    roscpp
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
  gl_wrapper
    src/render/render.cpp
    src/render/param.cpp
    src/render/display.cpp
    src/render/light.cpp
    src/render/camera.cpp
    src/render/material.cpp
    src/render/quaternion.cpp
    src/interface/mouse.cpp
    src/object/object.cpp
    src/object/x_object.cpp
    src/object/x_deformable_object.cpp
    src/object/x_hand.cpp
    src/object/x_right_hand.cpp
    src/object/x_left_hand.cpp
    src/object/simple_object.cpp
    src/object/grid.cpp
    src/object/coordinate.cpp
)

target_link_libraries(
  gl_wrapper
    GL
    glut
    GLU
    ${catkin_LIBRARIES}
)

add_executable(
  gl_wrapper_test
    test/test.cpp
)

target_link_libraries(
  gl_wrapper_test
    ${catkin_LIBRARIES}
    gl_wrapper
)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    gl_wrapper
    gl_wrapper_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)