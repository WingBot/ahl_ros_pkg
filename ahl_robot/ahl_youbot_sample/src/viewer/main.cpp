#include <stdexcept>
#include <ros/ros.h>
#include "ahl_youbot_sample/exception.hpp"
#include "ahl_youbot_sample/viewer/viewer.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "ahl_youbot_viewer");
    ros::NodeHandle nh;

    ahl_youbot::Viewer viewer;
    ros::spin();
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
    ROS_ERROR_STREAM("Unknown exception occured.");
    return -1;
  }

  return 0;
}
