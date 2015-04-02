#include <stdexcept>
#include <ros/ros.h>
#include "ahl_robot_controller/ahl_robot_controller.hpp"
#include "ahl_robot_controller/exceptions.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ahl_robot_controller_test");
  ros::NodeHandle nh;

  try
  {
    using namespace ahl_robot;
    AHLRobotControllerPtr robot_controller;
    robot_controller = AHLRobotControllerPtr(new AHLRobotController());

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
  }
  catch(ahl_robot::FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception occured.");
    exit(1);
  }

  return 0;
}
