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

#include "ahl_youbot_server/exceptions.hpp"
#include "ahl_youbot_server/interface/gazebo_interface.hpp"

using namespace ahl_youbot;

GazeboInterface::GazeboInterface(const std::vector<std::string>& joint_list, double servo_period)
  : subscribed_joint_states_(false)
{
  dof_ = joint_list.size();
  for(unsigned int i = 0; i < dof_; ++i)
  {
    name_to_idx_[joint_list[i]] = i;
    joint_list_.push_back(joint_list[i]);
    q_[joint_list[i]]   = 0.0;
    dq_[joint_list[i]]  = 0.0;
    effort_.name.push_back(joint_list[i]);
  }
  effort_.effort.resize(effort_.name.size());
  effort_.start_time = ros::Time(0);
  effort_.duration = ros::Duration(servo_period);

  ros::NodeHandle nh;
  sub_joint_states_ = nh.subscribe("/gazebo/joint_states", 10, &GazeboInterface::jointStatesCB, this);
  pub_apply_joint_efforts_ = nh.advertise<gazebo_msgs::ApplyJointEfforts>("/gazebo/apply_joint_efforts", 10);
}

bool GazeboInterface::getJointStates(Eigen::VectorXd& q)
{
  if(!subscribed_joint_states_)
  {
    return false;
  }

  q.resize(joint_list_.size());

  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(q_.find(joint_list_[i]) != q_.end())
    {
      q.coeffRef(i) = q_[joint_list_[i]];
    }
    else
    {
      return false;
    }
  }

  return true;
}

bool GazeboInterface::getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq)
{
  if(!subscribed_joint_states_)
  {
    return false;
  }

  q.resize(joint_list_.size());
  dq.resize(joint_list_.size());

  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(q_.find(joint_list_[i]) != q_.end())
    {
      q.coeffRef(i) = q_[joint_list_[i]];
    }
    else
    {
      return false;
    }

    if(dq_.find(joint_list_[i]) != dq_.end())
    {
      dq.coeffRef(i) = dq_[joint_list_[i]];
    }
    else
    {
      return false;
    }
  }
}

bool GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  if(tau.rows() != effort_.name.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != effort_.name.size()" << std::endl
        << "  tau.rows          : " << tau.rows() << std::endl
        << "  effort_.name.size : " << effort_.name.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }
  else if(tau.rows() != effort_.effort.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != effort_.effort.size()" << std::endl
        << "  tau.rows            : " << tau.rows() << std::endl
        << "  effort_.effort.size : " << effort_.effort.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }
  else if(tau.rows() != joint_list_.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != joint_list_.size()" << std::endl
        << "  tau.rows            : " << tau.rows() << std::endl
        << "  joint_list_.size : " << joint_list_.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }

  for(unsigned int i = 0; i < tau.rows(); ++i)
  {
    if(name_to_idx_.find(joint_list_[i]) == name_to_idx_.end())
    {
      return false;
    }

    int idx = name_to_idx_[joint_list_[i]];

    if(idx > effort_.effort.size() - 1)
    {
      return false;
    }

    effort_.effort[idx] = tau.coeff(i);
  }

  pub_apply_joint_efforts_.publish(effort_);
}

void GazeboInterface::jointStatesCB(const gazebo_msgs::JointStates::ConstPtr& msg)
{
  subscribed_joint_states_ = true;

  for(unsigned int i = 0; i < msg->name.size(); ++i)
  {
    q_[msg->name[i]]  = msg->q[i];
    dq_[msg->name[i]] = msg->dq[i];
  }
}
