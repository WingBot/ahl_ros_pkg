cmake_minimum_required(VERSION 2.8.3)
project(train_with_cg)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    cv_wrapper
    gl_wrapper
    roscpp
    std_utils
    tf
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
  CATKIN_DEPENDS
    cv_bridge
    cv_wrapper
    gl_wrapper
    roscpp
    std_utils
    tf
  DEPENDS
   Eigen
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

##############################################################################
# Build
##############################################################################

add_executable(
  image_collector
    src/collect_image/main.cpp
    src/collect_image/display.cpp
    src/collect_image/hand_image_collector.cpp
    src/collect_image/hand_pose.cpp
    src/collect_image/fingers.cpp
    src/collect_image/quaternion.cpp
    src/collect_image/zxy_euler_angles.cpp
    src/collect_image/depth_image_saver.cpp
)

target_link_libraries(
  image_collector
    ${catkin_LIBRARIES}
)

add_executable(
  train_with_cg
    src/train/main.cpp
    src/train/train.cpp
)

target_link_libraries(
  train_with_cg
    ${catkin_LIBRARIES}
)


##############################################################################
# Install
##############################################################################

install(
  TARGETS
    image_collector
    train_with_cg
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)