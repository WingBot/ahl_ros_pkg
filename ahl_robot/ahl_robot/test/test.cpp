#include <ros/ros.h>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/parser/parser.hpp"
#include "ahl_robot/parser/sdf_parser.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"

using namespace ahl_robot;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
#ifdef YOUBOT
    std::string name = "youbot";
    std::string base_name = "base_footprint";
    std::map<std::string, std::string> mnp_and_ee;
    mnp_and_ee["arm0"] = "gripper_palm_link";
    RobotPtr robot = RobotPtr(new Robot(name, base_name, mnp_and_ee));

    ParserPtr parser = ParserPtr(new SDFParser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/models/youbot/model.sdf";
    parser->load(path, robot);
#else
    std::string name = "pr2";
    std::string base_name = "base_footprint";
    std::map<std::string, std::string> mnp_and_ee;
    mnp_and_ee["left_arm"] = "l_wrist_roll_link";
    mnp_and_ee["right_arm"] = "r_wrist_roll_link";
    RobotPtr robot = RobotPtr(new Robot(name, base_name, mnp_and_ee));

    ParserPtr parser = ParserPtr(new SDFParser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/models/pr2/model.sdf";
    parser->load(path, robot);
#endif
    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = TfPublisherPtr(new TfPublisher());

    while(ros::ok())
    {
      tf_publisher->publish(robot);
      ros::Duration(0.1).sleep();
    }
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
