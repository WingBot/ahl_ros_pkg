cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_srvs)

find_package(
  catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_srvs
)

add_service_files(
  DIRECTORY
    srv
  FILES
    Float.srv
    SetJoint.srv
    JointSpaceControl.srv
    TaskSpaceControl.srv
    TaskSpaceHybridControl.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_srvs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_srvs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
