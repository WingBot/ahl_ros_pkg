cmake_minimum_required(VERSION 2.8.3)
project(std_utils)

##############################################################################
# Find dependencies
##############################################################################

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
  ${Eigen_ILBRARY_DIRS}
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
    std_utils
  CATKIN_DEPENDS
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
  std_utils
    src/str_utils.cpp
    src/io_utils.cpp
    src/yaml_utils.cpp
    src/yaml_loader.cpp
)

target_link_libraries(
  std_utils
    ${catkin_LIBRARIES}
    yaml-cpp
)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    std_utils
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)