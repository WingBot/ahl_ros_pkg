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
    subscribed_joint_states_(true)
{
  ros::NodeHandle local_nh("~");
  std::string name;

  local_nh.param<std::string>("gazebo_joint_effort", name, "joint_effort");

  ros::NodeHandle nh;
  client_start_timer_ = nh.serviceClient<gazebo_msgs::StartTimer>("/gazebo/start_timer");
  client_add_joint_   = nh.serviceClient<gazebo_msgs::AddJoint>("/gazebo/add_joint");
}

GazeboInterface::~GazeboInterface()
{
  this->applyJointEfforts(Eigen::VectorXd::Zero(joint_list_.size()));
}

void GazeboInterface::addJoint(const std::string& name, double effort_time)
{
  joint_map_[name] = 0.0;

  if(joint_to_idx_.find(name) == joint_to_idx_.end())
  {
    unsigned int size = joint_to_idx_.size();
    joint_to_idx_[name] = size;
    joint_list_.push_back(name);

    joint_effort_[name] = ahl_utils::SharedMemory<double>::Ptr(new ahl_utils::SharedMemory<double>(name + "::effort"));
    joint_state_[name]  = ahl_utils::SharedMemory<double>::Ptr(new ahl_utils::SharedMemory<double>(name + "::state"));

    gazebo_msgs::AddJoint srv;
    srv.request.name = name;
    srv.request.effort_time = effort_time;
    if(!client_add_joint_.call(srv))
    {
      std::stringstream msg;
      msg << "Could not add joint : " << name;
      throw ahl_gazebo_if::Exception("GazeboInterface::addJoint", msg.str());
    }
  }
}

void GazeboInterface::setDuration(double duration)
{
  duration_ = ros::Duration(duration);
}

void GazeboInterface::connect()
{
  joint_num_ = joint_map_.size();
  q_ = Eigen::VectorXd::Zero(joint_num_);

  gazebo_msgs::StartTimer srv;
  if(!client_start_timer_.call(srv))
  {
    throw ahl_gazebo_if::Exception("GazeboInterface::connect", "Could not start timer.");
  }

  ros::NodeHandle nh;
  pub_link_states_ = nh.advertise<gazebo_msgs::LinkStates>("/gazebo/set_link_states", 1);
}

bool GazeboInterface::subscribed()
{
  boost::mutex::scoped_lock lock(mutex_);
  return subscribed_joint_states_;
}

void GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  if(tau.rows() != joint_list_.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != joint_list_.size() + joint_idx_offset_" << std::endl
        << "  tau.rows()        : " << tau.rows() << std::endl
        << "  joint_list_.size() : " << joint_list_.size();

    throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEfforts", msg.str());
  }

  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(joint_to_idx_.find(joint_list_[i]) == joint_to_idx_.end())
    {
      std::stringstream msg;
      msg << joint_list_[i] << " was not found in joint_to_idx_.";
      throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEffots", msg.str());
    }

    joint_effort_[joint_list_[i]]->write(tau[i]);
  }
}

void GazeboInterface::addLink(const std::string& robot, const std::string& link)
{
  std::string name = robot + "::" + link;

  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.z = 1.0;

  geometry_msgs::Twist twist;
  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.linear.z = 0.0;
  twist.angular.x = 0.0;
  twist.angular.y = 0.0;
  twist.angular.z = 0.0;

  link_states_.name.push_back(name);
  link_states_.pose.push_back(pose);
  link_states_.twist.push_back(twist);
  link_states_.reference_frame.push_back(link);
}

void GazeboInterface::translateLink(const std::vector<Eigen::Vector3d>& p)
{
  if(p.size() != link_states_.name.size())
  {
    std::stringstream msg;
    msg << "p.size() != link_states_.name.size()" << std::endl
        << "  p.size : " << p.size() << std::endl
        << "  link_states_.name.size : " << link_states_.name.size();
    throw ahl_gazebo_if::Exception("GazeboInterface::translateLink", msg.str());
  }

  for(unsigned int i = 0; i < p.size(); ++i)
  {
    link_states_.pose[i].position.x = p[i].coeff(0);
    link_states_.pose[i].position.y = p[i].coeff(1);
    link_states_.pose[i].position.z = p[i].coeff(2);

    link_states_.pose[i].orientation.x = 0.0;
    link_states_.pose[i].orientation.y = 0.0;
    link_states_.pose[i].orientation.z = 0.0;
    link_states_.pose[i].orientation.w = 1.0;
  }

  pub_link_states_.publish(link_states_);
}

void GazeboInterface::rotateLink(const std::vector<Eigen::Quaternion<double> >& q)
{
  if(q.size() != link_states_.name.size())
  {
    std::stringstream msg;
    msg << "q.size() != link_states_.name.size()" << std::endl
        << "  q.size : " << q.size() << std::endl
        << "  link_states_.name.size : " << link_states_.name.size();
    throw ahl_gazebo_if::Exception("GazeboInterface::rotateLink", msg.str());
  }

  for(unsigned int i = 0; i < q.size(); ++i)
  {
    link_states_.pose[i].position.x = 0.0;
    link_states_.pose[i].position.y = 0.0;
    link_states_.pose[i].position.z = 0.0;

    link_states_.pose[i].orientation.x = q[i].x();
    link_states_.pose[i].orientation.y = q[i].y();
    link_states_.pose[i].orientation.z = q[i].z();
    link_states_.pose[i].orientation.w = q[i].w();
  }

  pub_link_states_.publish(link_states_);
}

const Eigen::VectorXd& GazeboInterface::getJointStates()
{
  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(joint_to_idx_.find(joint_list_[i]) == joint_to_idx_.end())
    {
      std::stringstream msg;
      msg << joint_list_[i] << " was not found in joint_to_idx_.";
      throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::getJointStates", msg.str());
    }

    joint_state_[joint_list_[i]]->read(q_[i]);
  }

  boost::mutex::scoped_lock lock(mutex_);
  return q_;
}
