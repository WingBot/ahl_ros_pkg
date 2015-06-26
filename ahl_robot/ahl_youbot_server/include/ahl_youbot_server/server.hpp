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

#ifndef __AHL_YOUBOT_SERVER_SERVER_HPP
#define __AHL_YOUBOT_SERVER_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/state/state.hpp"
#include "ahl_youbot_server/action/action_server.hpp"

namespace ahl_youbot
{

  class Server
  {
  public:
    Server();

  private:
    bool cancelCB(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    bool floatCB(
      ahl_robot_srvs::Float::Request& req,
      ahl_robot_srvs::Float::Response& res);
    bool setJointCB(
      ahl_robot_srvs::SetJoint::Request& req,
      ahl_robot_srvs::SetJoint::Response& res);
    bool jointSpaceControlCB(
      ahl_robot_srvs::JointSpaceControl::Request& req,
      ahl_robot_srvs::JointSpaceControl::Response& res);
    bool taskSpaceControlCB(
      ahl_robot_srvs::TaskSpaceControl::Request& req,
      ahl_robot_srvs::TaskSpaceControl::Response& res);
    bool taskSpaceHybridControlCB(
      ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
      ahl_robot_srvs::TaskSpaceHybridControl::Response& res);

    std::map<State::Type, StatePtr> state_;
    State::Type state_type_;

    ros::ServiceServer ros_server_change_state_;
    ros::ServiceServer ros_server_cancel_;
    ros::ServiceServer ros_server_float_;
    ros::ServiceServer ros_server_set_joint_;
    ros::ServiceServer ros_server_joint_space_control_;
    ros::ServiceServer ros_server_task_space_control_;
    ros::ServiceServer ros_server_task_space_hybrid_control_;

    ActionServerPtr action_server_;
  };

  typedef boost::shared_ptr<Server> ServerPtr;
};

#endif /* __AHL_YOUBOT_SERVER_SERVER_HPP */
