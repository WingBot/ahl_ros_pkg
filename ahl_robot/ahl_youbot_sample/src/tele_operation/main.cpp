#include <stdexcept>
#include <ros/ros.h>
#include "ahl_youbot_sample/exception.hpp"
#include "ahl_youbot_sample/tele_operation/tele_operation.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "ahl_youbot_teleoperation");
    ros::NodeHandle nh;

    ahl_youbot::TeleOperation tele_operation;
    tele_operation.run();
  }
  catch(ahl_youbot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(ros::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception occurred.");
    return -1;
  }

  return 0;
}
