#include <ros/ros.h>
#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/ahl_robot_controller.hpp"
#include "ahl_robot_controller/robot/xml_parser.hpp"

using namespace ahl_robot;

AHLRobotController::AHLRobotController()
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  double duration;

  std::string model_file_name;
  std::string robot_name;
  std::string base_frame;
  std::string operational_frame;
  std::string parent_frame;

  local_nh.param<std::string>("robot/model_file", model_file_name, "model.sdf");
  local_nh.param<std::string>("robot/name", robot_name, "robot");
  local_nh.param<std::string>("robot/frame/base", base_frame, "base_footprint");
  local_nh.param<std::string>("robot/operational_point/child/frame", operational_frame, "operational_point");
  local_nh.param<std::string>("robot/operational_point/parent/frame", parent_frame, "parent");

  Eigen::Vector3d xyz;
  local_nh.param<double>("robot/operational_point/parent/x", xyz.coeffRef(0), 0.0);
  local_nh.param<double>("robot/operational_point/parent/y", xyz.coeffRef(1), 0.0);
  local_nh.param<double>("robot/operational_point/parent/z", xyz.coeffRef(2), 0.0);

  Eigen::Vector3d rpy;
  local_nh.param<double>("robot/operational_point/parent/roll", rpy.coeffRef(0), 0.0);
  local_nh.param<double>("robot/operational_point/parent/pitch", rpy.coeffRef(1), 0.0);
  local_nh.param<double>("robot/operational_point/parent/yaw", rpy.coeffRef(2), 0.0);

  parser_ = ParserPtr(new XMLParser());
  robot_ = RobotPtr(new Robot());

  parser_->load(model_file_name, robot_name, robot_);

  robot_->setBaseFrame(base_frame);
  robot_->setOperatedFrame(operational_frame, parent_frame, xyz, rpy);
  robot_->setup();

  ManipulatorPtr manipulator = robot_->getManipulator();

  local_nh.param("tf/publish_joint_frames", publish_joint_frames_, true);
  local_nh.param("tf/publish_link_frames", publish_link_frames_, true);
  local_nh.param("tf/duration", duration, 0.05);

  if(publish_joint_frames_ || publish_link_frames_)
  {
    tf_publisher_ = TfPublisherPtr(new TfPublisher());
    timer_publish_tf_ = nh.createTimer(ros::Duration(duration), &AHLRobotController::publishTfCB, this);
  }
}

void AHLRobotController::publishTfCB(const ros::TimerEvent&)
{
  if(publish_joint_frames_)
    tf_publisher_->publishJointFrames(robot_);
  if(publish_link_frames_)
    tf_publisher_->publishLinkFrames(robot_);
}
