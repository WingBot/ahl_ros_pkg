cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_controller)

find_package(
  catkin REQUIRED COMPONENTS
    ahl_robot
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  Eigen REQUIRED
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   gazebo_msgs
# )

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_robot_controller
  CATKIN_DEPENDS
    ahl_robot
    dynamic_reconfigure
    gazebo_msgs
    roscpp
    rospy
  DEPENDS
    Eigen
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_definitions(
  ${EIGEN_DEFINITIONS}
)

add_library(ahl_robot_controller
  src/robot_controller.cpp
  src/task/joint_control.cpp
  src/task/gravity_compensation.cpp
)

target_link_libraries(
  ahl_robot_controller
    ${catkin_LIBRARIES}
)
