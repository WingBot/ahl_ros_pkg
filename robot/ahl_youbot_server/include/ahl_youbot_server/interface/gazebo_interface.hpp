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

#ifndef __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP
#define __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <gazebo_msgs/JointStates.h>
#include <gazebo_msgs/ApplyJointEfforts.h>
#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  class GazeboInterface : public Interface
  {
  public:
    GazeboInterface(const std::vector<std::string>& joint_list, double servo_period);

    bool getJointStates(Eigen::VectorXd& q);
    bool getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq);
    bool getJointEfforts(Eigen::VectorXd& tau);
    bool getOdometry(Eigen::Vector3d& q);
    bool getOdometry(Eigen::Vector3d& q, Eigen::Vector3d& dq);
    bool applyJointEfforts(const Eigen::VectorXd& tau);
    bool applyBaseVelocity(const Eigen::Vector3d& v);
  private:
    void jointStatesCB(const gazebo_msgs::JointStates::ConstPtr& msg);

    ros::Subscriber sub_joint_states_;
    ros::Publisher pub_apply_joint_efforts_;

    bool subscribed_joint_states_;

    unsigned int dof_;
    std::vector<std::string> joint_list_;
    std::map<std::string, int> name_to_idx_;
    std::map<std::string, double> q_;
    std::map<std::string, double> dq_;
    gazebo_msgs::ApplyJointEfforts effort_;
    ros::Time start_time_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP */
