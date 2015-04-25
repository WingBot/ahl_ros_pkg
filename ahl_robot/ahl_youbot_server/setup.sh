#!/bin/sh
MY_CATKIN_WS=$HOME/Work/catkin_ws

sudo setcap cap_net_raw+ep $MY_CATKIN_WS/devel/lib/ahl_youbot_server/ahl_youbot_server
