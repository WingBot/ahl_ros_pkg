cmake_minimum_required(VERSION 2.8.3)
project(ahl_rrt)

find_package(
  catkin REQUIRED COMPONENTS
    gl_wrapper
    message_generation
    roscpp
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

find_package(
  OpenMP REQUIRED
)

if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "$(CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

add_message_files(
  DIRECTORY
    msg
  FILES
    Edge2D.msg
    Edge3D.msg
)

generate_messages(
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
    ahl_rrt
  CATKIN_DEPENDS
    gl_wrapper
    roscpp
  DEPENDS
    Eigen
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

add_library(
  ahl_rrt
    src/rrt.cpp
    src/param.cpp
    src/tree/vertex.cpp
    src/tree/random_tree.cpp
)

target_link_libraries(
  ahl_rrt
    ${catkin_LIBRARIES}
)

add_executable(
  ahl_rrt_test
    test/test.cpp
)

add_dependencies(
  ahl_rrt_test
    ahl_rrt_gencpp
)

target_link_libraries(
  ahl_rrt_test
    ahl_rrt
)

add_executable(
  ahl_rrt_visualization
    test/visualization.cpp
)

add_dependencies(
  ahl_rrt_visualization
    ahl_rrt_gencpp
)

target_link_libraries(
  ahl_rrt_visualization
    ahl_rrt
)
