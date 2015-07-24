#include "ahl_robot_samples/pr2/pr2.hpp"

using namespace ahl_sample;

PR2::PR2()
{

}

void PR2::init()
{
  initRobot("pr2", "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot_samples/yaml/pr2.yaml");

  controller_ = RobotControllerPtr(new RobotController());
  controller_->init(robot_, "left_mnp");

  gazebo_interface_ = GazeboInterfacePtr(new GazeboInterface());
  gazebo_interface_->setDuration(0.010);
  gazebo_interface_->addJoint("pr2::base_x_joint");
  gazebo_interface_->addJoint("pr2::base_y_joint");
  gazebo_interface_->addJoint("pr2::base_yaw_joint");
  gazebo_interface_->addJoint("pr2::shoulder_pan_l_joint");
  gazebo_interface_->addJoint("pr2::shoulder_lift_l_joint");
  gazebo_interface_->addJoint("pr2::upper_arm_l_joint");
  gazebo_interface_->addJoint("pr2::elbow_l_joint");
  gazebo_interface_->addJoint("pr2::forearm_l_joint");
  gazebo_interface_->addJoint("pr2::wrist_flex_l_joint");
  gazebo_interface_->addJoint("pr2::wrist_roll_l_joint");
  gazebo_interface_->addJoint("pr2::shoulder_pan_r_joint");
  gazebo_interface_->addJoint("pr2::shoulder_lift_r_joint");
  gazebo_interface_->addJoint("pr2::upper_arm_r_joint");
  gazebo_interface_->addJoint("pr2::elbow_r_joint");
  gazebo_interface_->addJoint("pr2::forearm_r_joint");
  gazebo_interface_->addJoint("pr2::wrist_flex_r_joint");
  gazebo_interface_->addJoint("pr2::wrist_roll_r_joint");
  gazebo_interface_->connect();

  tf_pub_ = TfPublisherPtr(new TfPublisher());
}

void PR2::run()
{
  ros::NodeHandle nh;

  timer_update_model_ = nh.createTimer(ros::Duration(0.01), &PR2::updateModel, this);
  timer_control_ = nh.createTimer(ros::Duration(0.001), &PR2::control, this);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}

void PR2::updateModel(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(mutex_);

}

void PR2::control(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(mutex_);

}
