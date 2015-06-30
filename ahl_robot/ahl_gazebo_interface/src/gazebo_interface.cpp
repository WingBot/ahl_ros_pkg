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

#include "ahl_gazebo_interface/gazebo_interface.hpp"
#include "ahl_gazebo_interface/exception.hpp"

using namespace ahl_gazebo_if;

GazeboInterface::GazeboInterface()
  : duration_(ros::Duration(0.1)),
    subscribed_joint_states_(false),
    joint_idx_offset_(0),
    use_mobility_2d_(false),
    subscribed_model_states_(true)
{
}

void GazeboInterface::addJoint(const std::string& name)
{
  q_map_[name] = 0.0;
  if(name_to_idx_.find(name) == name_to_idx_.end())
  {
    unsigned int size = name_to_idx_.size();
    name_to_idx_[name] = size;
    name_list_.push_back(name);
  }
}

void GazeboInterface::addMobility2D(const std::string& robot)
{
  robot_ = robot;
  model_state_.model_name = robot;
  joint_idx_offset_ = 3;
  use_mobility_2d_ = true;
  subscribed_model_states_ = false;
}

void GazeboInterface::setDuration(double duration)
{
  duration_ = ros::Duration(duration);
}

void GazeboInterface::connect()
{
  joint_num_ = q_map_.size() + joint_idx_offset_;
  q_ = Eigen::VectorXd::Zero(joint_num_);
  effort_.start_time = ros::Time(0);
  effort_.duration = duration_;
  effort_.effort.resize(q_map_.size());
  effort_.name.resize(q_map_.size());
  std::map<std::string, int>::iterator it;
  for(it = name_to_idx_.begin(); it != name_to_idx_.end(); ++it)
  {
    effort_.name[it->second] = it->first;
  }

  ros::NodeHandle nh;
  pub_effort_ = nh.advertise<gazebo_msgs::ApplyJointEfforts>(
    TOPIC_PUB_JOINT_EFFORT, 10);
  sub_joint_states_ = nh.subscribe(
    TOPIC_SUB_JOINT_STATES, 10, &GazeboInterface::subscribeJointStates, this);
  if(use_mobility_2d_)
  {
    pub_model_state_ = nh.advertise<gazebo_msgs::ModelState>(
      TOPIC_PUB_MODEL_STATE, 10);
    sub_model_states_ = nh.subscribe(
      TOPIC_SUB_MODEL_STATES, 10, &GazeboInterface::subscribeModelStates, this);
  }
}

bool GazeboInterface::subscribed()
{
  boost::mutex::scoped_lock lock(mutex_);
  return (subscribed_joint_states_ & subscribed_model_states_);
}

void GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  if(tau.rows() != name_list_.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != name_list_.size()" << std::endl
        << "  tau.rows()        : " << tau.rows() << std::endl
        << "  name_list_.size() : " << name_list_.size();

    throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEfforts", msg.str());
  }

  for(unsigned int i = 0; i < tau.rows(); ++i)
  {
    if(name_to_idx_.find(name_list_[i]) == name_to_idx_.end())
    {
      std::stringstream msg;
      msg << name_list_[i] << " was not found in name_to_idx_.";
      throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEffots", msg.str());
    }

    effort_.effort[name_to_idx_[name_list_[i]]] = tau.coeff(i);
  }

  pub_effort_.publish(effort_);
}

void GazeboInterface::applyVelocityCommand2D(const Eigen::Vector3d& v)
{
  if(!use_mobility_2d_)
  {
    std::stringstream msg;
    msg << "Called method to use 2D mobility, however 2D mobility is not set.";
    throw ahl_gazebo_if::Exception("GazeboInterface::applyVelocityCommand", msg.str());
  }

  model_state_.twist.linear.x  = v.coeff(0);
  model_state_.twist.linear.y  = v.coeff(1);
  model_state_.twist.angular.x = 0.0;
  model_state_.twist.angular.y = 0.0;
  model_state_.twist.angular.z = v.coeff(2);
}

void GazeboInterface::subscribeJointStates(const gazebo_msgs::JointStates::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  for(unsigned int i = 0; i < msg->name.size(); ++i)
  {
    q_map_[msg->name[i]] = msg->q[i];
  }

  for(unsigned int i = 0; i < name_list_.size(); ++i)
  {
    if(q_map_.find(name_list_[i]) != q_map_.end())
    {
      q_.coeffRef(i + joint_idx_offset_) = q_map_[name_list_[i]];
    }
  }

  subscribed_joint_states_ = true;
}

void GazeboInterface::subscribeModelStates(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  if(use_mobility_2d_)
  {
    for(unsigned int i = 0; i < msg->name.size(); ++i)
    {
      if(msg->name[i] == robot_)
      {
        q_.coeffRef(0) = msg->pose[i].position.x;
        q_.coeffRef(1) = msg->pose[i].position.y;

        double rad = 2.0 * acos(msg->pose[i].orientation.w);
        if(msg->pose[i].orientation.z < 0.0)
        {
          rad *= -1.0;
        }
        q_.coeffRef(2) = atan2(sin(rad), cos(rad));

        model_state_.pose = msg->pose[i];
        model_state_.twist = msg->twist[i];

        subscribed_model_states_ = true;
        return;
      }
    }

    std::stringstream msg;
    msg << "Could not find model : " << robot_ << std::endl;
    throw ahl_gazebo_if::Exception("GazeboInterface::subscribeModelStates", msg.str());
  }
}
