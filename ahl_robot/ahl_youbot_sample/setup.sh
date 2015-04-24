#!/bin/sh
CATKIN_WS=/home/daichi/Work/catkin_ws
YOUBOT_DRIVER_PATH=$CATKIN_WS/src/ahl_ros_pkg/ahl_robot/youbot/youbot_driver

sudo setcap cap_net_raw+ep $CATKIN_WS/devel/lib/hello_world_demo/youBot_HelloWorldDemo
sudo setcap cap_net_raw+ep $CATKIN_WS/devel/lib/ahl_youbot_sample/ahl_youbot_tele_operation
sudo setcap cap_net_raw+ep $CATKIN_WS/devel/lib/ahl_youbot_sample/ahl_youbot_viewer


