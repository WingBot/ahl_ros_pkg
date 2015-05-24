cmake_minimum_required(VERSION 2.8.3)
project(ahl_robot)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    std_utils
    tf2
    tf2_ros
)

find_package(
  cmake_modules REQUIRED
)

find_package(PkgConfig)

pkg_check_modules(NEW_YAMLcpp yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

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
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ahl_robot
  CATKIN_DEPENDS
    roscpp
    std_utils
    tf2
    tf2_ros
  DEPENDS
    Eigen
    yaml-cpp
)

add_library(
  ahl_robot
    src/robot/manipulator.cpp
    src/robot/transformation.cpp
    src/robot/parser.cpp
    src/tf/tf_publisher.cpp
    src/utils/math.cpp
)

target_link_libraries(
  ahl_robot
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(
  parser_test
    test/test.cpp
)

add_dependencies(
  parser_test
    ahl_robot_gencpp
)

target_link_libraries(
  parser_test
    ahl_robot
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
# install(TARGETS ahl_robot ahl_robot_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ahl_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
