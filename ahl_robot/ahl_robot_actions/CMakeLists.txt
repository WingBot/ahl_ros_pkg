cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_actions)

find_package(
  catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_generation
)

add_action_files(
  DIRECTORY
    action
  FILES
    Float.action
    JointSpaceControl.action
    SetJoint.action
    TaskSpaceControl.action
    TaskSpaceHybridControl.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
