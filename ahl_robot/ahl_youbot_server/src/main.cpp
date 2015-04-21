#include <ros/ros.h>
#include "ahl_youbot_server/server.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ahl_youbot_server");
  ros::NodeHandle nh;

  using namespace ahl_youbot;
  ServerPtr server = ServerPtr(new Server());

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
