cmake_minimum_required(VERSION 2.8.3)
project(neural_network)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    std_utils
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

find_package(
  OpenMP
)

if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "$(CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

pkg_check_modules(NEW_YAMLcpp yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

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
    neural_network
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
  neural_network
    src/config.cpp
    src/training_data.cpp
    src/scaler.cpp
    src/core/neural_network.cpp
    src/core/layer.cpp
    src/core/forward_calculator.cpp
    src/core/backward_calculator.cpp
    src/core/multi_threaded_back_propagation.cpp
    src/core/back_propagation.cpp
    src/core/sigmoid.cpp
    src/core/sigmoid_table.cpp
    src/core/piecewise_linear.cpp
    src/core/rectified_linear.cpp
)

target_link_libraries(
  neural_network
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(
  nn_test
    test/test.cpp
)

target_link_libraries(
  nn_test
    ${catkin_LIBRARIES}
    neural_network
)

##############################################################################
# Install
##############################################################################

install(
  TARGETS
    neural_network
    nn_test
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
