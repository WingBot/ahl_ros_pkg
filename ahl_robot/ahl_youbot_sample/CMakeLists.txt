cmake_minimum_required(VERSION 2.8.3)
project(ahl_youbot_sample)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    youbot_driver
)

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_CONFIG_DIR}")

catkin_package(
  CATKIN_DEPENDS
    roscpp
    youbot_driver
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  tele_operation
    src/tele_operation/main.cpp
    src/tele_operation/tele_operation.cpp
)

target_link_libraries(
  tele_operation
    ${catkin_LIBRARIES}
    ncurses
)

add_executable(
  viewer
    src/viewer/main.cpp
    src/viewer/viewer.cpp
)

target_link_libraries(
  viewer
    ${catkin_LIBRARIES}
)

