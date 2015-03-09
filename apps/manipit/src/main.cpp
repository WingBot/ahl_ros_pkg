#include <stdexcept>
#include <ros/ros.h>
#include "manipit/manipit_server.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "manipit");
    ros::NodeHandle nh;

    using namespace manipit;
    ManipitServerPtr manipit_server = ManipitServerPtr(new ManipitServer());

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
    return -1;
  }

  return 0;
}
