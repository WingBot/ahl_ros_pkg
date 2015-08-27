cmake_minimum_required(VERSION 2.8.3)
project(cv_wrapper)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

find_package(
  OpenCV REQUIRED
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    cv_wrapper
  CATKIN_DEPENDS
    cv_bridge
    image_transport
    message_filters
    roscpp
    std_srvs
)

##############################################################################
# Build
##############################################################################

add_library(
  cv_wrapper
    src/image_subscriber.cpp
    src/utils.cpp
    src/rect.cpp
    src/hsv_mask.cpp
)

target_link_libraries(
  cv_wrapper
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

#add_executable(
#  cv_wrapper_test
#    test/test.cpp
#)

#target_link_libraries(
#  cv_wrapper_test
#    ${catkin_LIBRARIES}
#    cv_wrapper
#)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    cv_wrapper
#    cv_wrapper_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)