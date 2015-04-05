#include <ros/ros.h>
#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/ahl_robot_controller.hpp"
#include "ahl_robot_controller/robot/xml_parser.hpp"

using namespace ahl_robot;

AHLRobotController::AHLRobotController()
{
  parser_ = ParserPtr(new XMLParser());
  robot_ = RobotPtr(new Robot(std::string("base_link")));

  parser_->load("/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot_controller/sdf/model.sdf", "youbot", robot_);
  //parser_->load("/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot_controller/sdf/simple_arm.sdf", "simple_arm", robot_);

  robot_->print();
  //robot_->printNameList();

  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  double duration;

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
    ;//tf_publisher_->publishJointFrames(robot_);
  if(publish_link_frames_)
    tf_publisher_->publishLinkFrames(robot_);
}
