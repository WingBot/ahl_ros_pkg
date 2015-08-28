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
    neural_network
    roscpp
    ahl_utils
    tf
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

find_package(
  OpenCV REQUIRED
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    cv_wrapper
    gl_wrapper
    neural_network
    roscpp
    ahl_utils
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
    ${OpenCV_LIBS}
)

add_executable(
  auto_encoder
    src/auto_encoder/main.cpp
    src/auto_encoder/auto_encoder.cpp
)

target_link_libraries(
  auto_encoder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(
  filter_checker
    src/auto_encoder/filter_checker.cpp
)

target_link_libraries(
  filter_checker
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(
  decoder
    src/auto_encoder/decoder.cpp
)

target_link_libraries(
  decoder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(
  train_with_cg
    src/train/main.cpp
    src/train/train.cpp
)

target_link_libraries(
  train_with_cg
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


##############################################################################
# Install
##############################################################################

install(
  TARGETS
    image_collector
    auto_encoder
    filter_checker
    decoder
    train_with_cg
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)