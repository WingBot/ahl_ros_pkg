/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#include <stdexcept>
#include "ahl_youbot_server/exception.hpp"
#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"
#include "ahl_youbot_server/interface/youbot_interface.hpp"
#include "ahl_youbot_server/interface/gazebo_interface.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer(
  const std::string& robot_name, const std::string& mnp_name, const std::string& yaml,
  double period, double servo_period, bool use_real_robot)
  : mnp_name_(mnp_name)
{
  action_type_ = Action::FLOAT;

  // Initialize robot
  robot_ = ahl_robot::RobotPtr(new ahl_robot::Robot(robot_name));
  ahl_robot::ParserPtr parser = ahl_robot::ParserPtr(new ahl_robot::Parser());
  parser->load(yaml, robot_);

  // Initialize robot controller
  controller_ = ahl_ctrl::RobotControllerPtr(new ahl_ctrl::RobotController());
  controller_->init(robot_, mnp_name);

  // Initialize TfPublisher
  tf_pub_ = ahl_robot::TfPublisherPtr(new ahl_robot::TfPublisher());

  // Initialize interface
  if(use_real_robot)
  {
    ros::NodeHandle local_nh("~");

    std::string ahl_cfg_yaml = "";
    std::string cfg_path = "";
    bool do_calibration = true;

    local_nh.param<std::string>("interface/config/ahl_cfg_yaml", ahl_cfg_yaml, "");
    local_nh.param<std::string>("interface/config/path", cfg_path, "");
    local_nh.param<bool>("interface/config/do_calibration", do_calibration, true);

    interface_ = InterfacePtr(
      new YoubotInterface(robot_->getDOF(mnp_name_), ahl_cfg_yaml, cfg_path, do_calibration));
  }
  else
  {
    std::vector<std::string> joint_list;
    joint_list.push_back("youbot::arm_joint_1");
    joint_list.push_back("youbot::arm_joint_2");
    joint_list.push_back("youbot::arm_joint_3");
    joint_list.push_back("youbot::arm_joint_4");
    joint_list.push_back("youbot::arm_joint_5");
    interface_ = InterfacePtr(new GazeboInterface(joint_list, servo_period * 5.0));
  }

  // Initialize action
  action_[Action::FLOAT] = ActionPtr(
    new FloatAction("float", robot_, controller_, interface_));
  action_[Action::JOINT_SPACE_CONTROL] = ActionPtr(
    new JointSpaceControlAction("joint_space_control", robot_, controller_, interface_));
  action_[Action::TASK_SPACE_CONTROL] = ActionPtr(
    new TaskSpaceControlAction("task_space_control", robot_, controller_, interface_));
  action_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionPtr(
    new TaskSpaceHybridControlAction("task_space_hybrid_control", robot_));

  action_[action_type_]->init();

  this->updateRobotOnce();

  // Initialize timers
  ros::NodeHandle nh;

  timer_ = nh.createTimer(
    ros::Duration(period), &ActionServer::lowRateTimerCB, this);
  servo_timer_ = nh.createTimer(
    ros::Duration(servo_period), &ActionServer::highRateTimerCB, this);
}

void ActionServer::start()
{
  boost::mutex::scoped_lock lock(mutex_);
  this->updateRobotOnce();
  timer_.start();
  servo_timer_.start();
}

void ActionServer::stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  timer_.stop();
  servo_timer_.stop();
}

void ActionServer::switchActionTo(Action::Type action_type)
{
  boost::mutex::scoped_lock lock(mutex_);
  action_type_ = action_type;
  action_[action_type_]->init();
}

void ActionServer::updateRobotOnce()
{
  ros::Rate r(10.0);

  while(ros::ok())
  {
    if(this->updateMobilityOnce())
    {
      break;
    }
    else
    {
      ROS_INFO_STREAM("Waiting update of mobility ...");
    }

    r.sleep();
  }

  while(ros::ok())
  {
    if(this->updateManipulatorOnce())
    {
      break;
    }
    else
    {
      ROS_INFO_STREAM("Waiting update of manipulator ...");
    }

    r.sleep();
  }

  robot_->computeBasicJacobian(mnp_name_);
  robot_->computeMassMatrix(mnp_name_);
}

bool ActionServer::updateMobilityOnce()
{
  Eigen::Vector3d odom = Eigen::Vector3d::Zero();

  if(interface_->getOdometry(odom))
  {
    this->updateMobility(odom);
    return true;
  }
  else
  {
    return false;
  }
}

bool ActionServer::updateManipulatorOnce()
{
  Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_->getDOF(mnp_name_));

  if(interface_->getJointStates(q))
  {
    Eigen::Vector3d odom;

    this->copyOdometryTo(odom);
    q.block(0, 0, 3, 1) = odom;
    robot_->update(mnp_name_, q);

    return true;
  }
  else
  {
    return false;
  }
}

void ActionServer::convertYawToQuaternion(double yaw, Eigen::Quaternion<double>& q)
{
  q.x() = 0.0;
  q.y() = 0.0;
  q.z() = sin(0.5 * yaw);
  q.w() = cos(0.5 * yaw);
}

void ActionServer::convertQuaternionToYaw(const Eigen::Quaternion<double>& q, double& yaw)
{
  yaw = 2.0 * acos(q.w());
  yaw = atan2(sin(yaw), cos(yaw));

  if(q.z() < 0.0)
  {
    yaw *= -1.0;
  }
}

void ActionServer::updateMobility(const Eigen::Vector3d& odom)
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaternion<double> orientation;

  position.block(0, 0, 2, 1) = odom.block(0, 0, 2, 1);
  this->convertYawToQuaternion(odom[2], orientation);

  robot_->updateBase(position, orientation);
}

void ActionServer::copyOdometryTo(Eigen::Vector3d& odom)
{
  odom.block(0, 0, 2, 1) = robot_->getMobility()->p.block(0, 0, 2, 1);
  this->convertQuaternionToYaw(robot_->getMobility()->r, odom[2]);
}

void ActionServer::lowRateTimerCB(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);
    robot_->computeBasicJacobian(mnp_name_);
    robot_->computeMassMatrix(mnp_name_);
    controller_->updateModel();
    // apply base velocity

    tf_pub_->publish(robot_, false);
  }
  catch(ahl_youbot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception occurred.");
    return;
  }
}

void ActionServer::highRateTimerCB(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);

    // update mobility
    Eigen::Vector3d odom;
    interface_->getOdometry(odom);
    this->updateMobility(odom);

    // update joint angle
    Eigen::VectorXd q = Eigen::VectorXd::Zero(robot_->getDOF(mnp_name_));;
    interface_->getJointStates(q);
    q.block(0, 0, 3, 1) = odom;
    robot_->update(mnp_name_, q);

    // compute tau in execute method
    void* test;
    action_[action_type_]->execute(test);
  }
  catch(ahl_youbot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception occurred.");
    return;
  }
}
