#!/bin/sh
CATKIN_WS=/home/daichi/Work/catkin_ws
YOUBOT_DRIVER_PATH=$CATKIN_WS/src/ahl_ros_pkg/ahl_robot/youbot/youbot_driver

sudo setcap cap_net_raw+ep $CATKIN_WS/devel/lib/youbot_driver/base_arm_gripper_test
sudo setcap cap_net_raw+ep $CATKIN_WS/devel/lib/youbot_driver/displayIpAddress
export YOUBOT_CONFIG_FOLDER_LOCATION=$YOUBOT_DRIVER_PATH/config

