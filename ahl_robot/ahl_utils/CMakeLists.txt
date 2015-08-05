cmake_minimum_required(VERSION 2.8.3)
project(ahl_utils)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
)

find_package(
  cmake_modules REQUIRED
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  rt
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_utils
  CATKIN_DEPENDS
    roscpp
)

add_library(
  shm_utils
    src/shared_memory.cpp
)

target_link_libraries(
  shm_utils
   ${catkin_LIBRARIES}
   rt
)

add_executable(
  shm_writer
    test/shm_writer.cpp
)

target_link_libraries(
  shm_writer
    shm_utils
)

add_executable(
  shm_reader
    test/shm_reader.cpp
)

target_link_libraries(
  shm_reader
    shm_utils
)