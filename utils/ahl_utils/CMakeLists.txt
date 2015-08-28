cmake_minimum_required(VERSION 2.8.3)
project(ahl_utils)

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
    ahl_utils
  CATKIN_DEPENDS
    roscpp
  DEPENDS
    Eigen
)

add_library(
  ahl_utils
    src/shared_memory.cpp
    src/io_utils.cpp
    src/str_utils.cpp
    src/yaml_loader.cpp
    src/yaml_utils.cpp
)

target_link_libraries(
  ahl_utils
    ${catkin_LIBRARIES}
    rt
    yaml-cpp
)

add_executable(
  shm_writer
    test/shm_writer.cpp
)

target_link_libraries(
  shm_writer
    ahl_utils
)

add_executable(
  shm_reader
    test/shm_reader.cpp
)

target_link_libraries(
  shm_reader
    ahl_utils
)