cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot_controller)

##############################################################################
# Find dependencies
##############################################################################

find_package(
  catkin REQUIRED COMPONENTS
    dynamic_reconfigure
    geometry_msgs
    roscpp
    rospy
    std_utils
    tf2
    tf2_ros
)

find_package(
  cmake_modules REQUIRED
)

find_package(
  TinyXML REQUIRED
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
  ${TinyXML_INCLUDE_DIRS}
)

##############################################################################
# Add dynamic reconfigure api
##############################################################################

generate_dynamic_reconfigure_options(
  cfg/dynamic_param.cfg
)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_robot_controller
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    roscpp
    rospy
    std_utils
    tf2
    tf2_ros
  DEPENDS
    Eigen
    TinyXML
)

add_library(
  ahl_robot_controller
    src/ahl_robot_controller.cpp
    src/robot/robot.cpp
    src/robot/link.cpp
    src/robot/joint.cpp
    src/robot/xml_parser.cpp
    src/robot/tf_publisher.cpp
)

add_dependencies(
  ahl_robot_controller
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(
  ahl_robot_controller
    ${catkin_LIBRARIES}
    ${TinyXML_LIBRARIES}
)

add_executable(
  ahl_robot_controller_test
    test/test.cpp
)

target_link_libraries(
  ahl_robot_controller_test
    ahl_robot_controller
)

