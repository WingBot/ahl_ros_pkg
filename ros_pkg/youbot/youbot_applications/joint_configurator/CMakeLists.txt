cmake_minimum_required(VERSION 2.8.3)
project(joint_configurator)

find_package(
  catkin REQUIRED COMPONENTS
    roscpp
    youbot_driver
)

catkin_package(
  LIBRARIES
    joint_configurator
  CATKIN_DEPENDS
    roscpp
    youbot_driver
)

if(NOT DEFINED JOINT_CONFIGURATOR_CONFIG_DIR)
  set(JOINT_CONFIGURATOR_CONFIG_DIR ${PROJECT_SOURCE_DIR}/config/)
endif(NOT DEFINED JOINT_CONFIGURATOR_CONFIG_DIR)

add_definitions(-DYOUBOT_CONFIGURATIONS_DIR="${OODL_YOUBOT_CONFIG_DIR}")
add_definitions(-DCONFIG_DIR="${JOINT_CONFIGURATOR_CONFIG_DIR}")

set(CMAKE_C_FLAGS_DEBUG "-fPIC -Wall -g")
set(CMAKE_CXX_FLAGS_DEBUG "-fPIC -Wall -g")

set(
  YOUBOT_DRIVER_CONFIG
    ${PROJECT_SOURCE_DIR}/config/arm-1.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-2.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-3.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-4.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-5.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-1-protected.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-2-protected.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-3-protected.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-4-protected.cfg
    ${PROJECT_SOURCE_DIR}/config/arm-5-protected.cfg
    ${PROJECT_SOURCE_DIR}/config/base.cfg
    ${PROJECT_SOURCE_DIR}/config/base-protected.cfg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(
  joint_configurator
    src/JointConfigurator.cpp
)

target_link_libraries(
  joint_configurator
    ${catkin_LIBRARIES}
)

add_executable(
  youbot_joint_configurator
    src/JointConfiguratorMain.cpp
)

target_link_libraries(
  youbot_joint_configurator
    joint_configurator
)

add_executable(
  youbot_arm_configurator
    src/ArmConfigurator.cpp
)

target_link_libraries(
  youbot_arm_configurator
    joint_configurator
)

add_executable(
  youbot_base_configurator
    src/BaseConfigurator.cpp
)

target_link_libraries(
  youbot_base_configurator
    joint_configurator
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(joint_configurator_node joint_configurator_generate_messages_cpp)

## Specify libraries to link a library or executable target against

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
# install(TARGETS joint_configurator joint_configurator_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_joint_configurator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
