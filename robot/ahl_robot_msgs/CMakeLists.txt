cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_msgs)

find_package(
  catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    GeneralizedCoordinate.msg
    GeneralizedForce.msg
    RobotPose.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
