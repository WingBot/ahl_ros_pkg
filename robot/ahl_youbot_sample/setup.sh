#!/bin/sh
MY_CATKIN_WS=$HOME/Work/catkin_ws

sudo setcap cap_net_raw+ep $MY_CATKIN_WS/devel/lib/hello_world_demo/youBot_HelloWorldDemo
sudo setcap cap_net_raw+ep $MY_CATKIN_WS/devel/lib/ahl_youbot_sample/tele_operation
sudo setcap cap_net_raw+ep $MY_CATKIN_WS/devel/lib/ahl_youbot_sample/viewer
sudo setcap cap_net_raw+ep $MY_CATKIN_WS/devel/lib/ahl_youbot_sample/tuning
