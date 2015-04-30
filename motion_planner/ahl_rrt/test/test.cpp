#include <stdexcept>
#include <ros/ros.h>
#include "ahl_rrt/rrt.hpp"
#include "ahl_rrt/exception.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "ahl_rrt_test");
    ros::NodeHandle nh;

    using namespace ahl_rrt;
    RRTBasePtr rrt = RRTBasePtr(new RRT());

    ParamPtr param = ParamPtr(new Param());
    Eigen::Vector3d init_x;
    init_x << 0, 0, 0;

    rrt->init(param, init_x);

    Eigen::Vector3d dst_x;
    dst_x << 50, 50, 50;
    rrt->buildTree(dst_x);
  }
  catch(ros::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return -1;
  }
  catch(ahl_rrt::Exception& e)
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
  }

  return 0;
}
