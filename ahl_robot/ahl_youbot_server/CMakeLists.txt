cmake_minimum_required(VERSION 2.8.3)
project(ahl_youbot_server)

find_package(
  catkin REQUIRED COMPONENTS
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
    src/state/float.cpp
    src/state/move.cpp
    src/state/lock.cpp
    src/state/alarm.cpp
    src/action/action_server.cpp
    src/action/float_action.cpp
    src/action/set_joint_action.cpp
    src/action/joint_space_control_action.cpp
    src/action/task_space_control_action.cpp
    src/action/task_space_hybrid_control_action.cpp
    src/youbot/youbot.cpp
    src/youbot/real_youbot_base.cpp
    src/youbot/gazebo_youbot_base.cpp
    src/youbot/real_youbot_manipulator.cpp
    src/youbot/gazebo_youbot_manipulator.cpp
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

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ahl_youbot_server ahl_youbot_server_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ahl_youbot_server.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
