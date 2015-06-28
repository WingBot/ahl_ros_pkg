cmake_minimum_required(VERSION 2.8.3)
project(ahl_youbot_server)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_robot
    ahl_robot_actions
    ahl_robot_controller
    ahl_robot_srvs
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
    youbot_driver
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${youbot_driver_CONFIG_DIR}")

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    ahl_robot
    ahl_robot_actions
    ahl_robot_controller
    ahl_robot_srvs
    gazebo_msgs
    geometry_msgs
    roscpp
    tf2
    youbot_driver
  DEPENDS
    Eigen
)

add_executable(
  ahl_youbot_server
    src/main.cpp
    src/server.cpp
    src/state/state.cpp
    src/state/disabled.cpp
    src/state/ready.cpp
    src/state/move.cpp
    src/state/alarm.cpp
    src/action/action_server.cpp
    src/action/float_action.cpp
    src/action/joint_space_control_action.cpp
    src/action/task_space_control_action.cpp
    src/action/task_space_hybrid_control_action.cpp
    src/interface/youbot_interface.cpp
    src/interface/gazebo_interface.cpp
)

add_dependencies(
  ahl_youbot_server
    ahl_robot_srvs_gencpp
    ahl_robot_actions_gencpp
)

target_link_libraries(
  ahl_youbot_server
    ${catkin_LIBRARIES}
)

add_executable(
  youbot_test
    test/test.cpp
)

target_link_libraries(
  youbot_test
    ${catkin_LIBRARIES}
)
