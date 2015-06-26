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

#include "ahl_youbot_server/server.hpp"
#include "ahl_youbot_server/state/alarm.hpp"
#include "ahl_youbot_server/state/disabled.hpp"
#include "ahl_youbot_server/state/float.hpp"
#include "ahl_youbot_server/state/lock.hpp"
#include "ahl_youbot_server/state/move.hpp"
#include "ahl_youbot_server/state/ready.hpp"

using namespace ahl_youbot;

Server::Server()
{
  ros::NodeHandle local_nh("~");

  std::string robot_name = "";
  std::string mnp_name = "";
  std::string yaml = "";
  double period = 0.01;
  double servo_period = 0.005;
  bool use_real_robot = false;
  local_nh.param<std::string>("robot/name", robot_name, "youbot");
  local_nh.param<std::string>("robot/mnp_name", mnp_name, "");
  local_nh.param<std::string>("robot/yaml", yaml, "");
  local_nh.param<double>("robot/period", period, 0.01);
  local_nh.param<double>("robot/servo_period", servo_period, 0.005);
  local_nh.param<bool>("robot/real", use_real_robot, false);

  action_server_ = ActionServerPtr(
    new ActionServer(robot_name, mnp_name, yaml, period, servo_period, use_real_robot));

  state_[State::ALARM]    = StatePtr(
    new Alarm(state_type_, action_server_));
  state_[State::DISABLED] = StatePtr(
    new Disabled(state_type_, action_server_));
  state_[State::FLOAT]    = StatePtr(
    new Float(state_type_, action_server_));
  state_[State::LOCK]     = StatePtr(
    new Lock(state_type_, action_server_));
  state_[State::MOVE]     = StatePtr(
    new Move(state_type_, action_server_));
  state_[State::READY]    = StatePtr(
    new Ready(state_type_, action_server_));

  state_type_ = State::READY;

  ros_server_cancel_ = local_nh.advertiseService(
    "command/cancel", &Server::cancelCB, this);
  ros_server_float_ = local_nh.advertiseService(
    "command/float", &Server::floatCB, this);
  ros_server_set_joint_ = local_nh.advertiseService(
    "command/set_joint", &Server::setJointCB, this);
  ros_server_joint_space_control_ = local_nh.advertiseService(
    "command/joint_space_control", &Server::jointSpaceControlCB, this);
  ros_server_task_space_control_ = local_nh.advertiseService(
    "command/task_space_control", &Server::taskSpaceControlCB, this);
  ros_server_task_space_hybrid_control_ = local_nh.advertiseService(
    "command/task_space_hybrid_control", &Server::taskSpaceHybridControlCB, this);
}

bool Server::cancelCB(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  return state_[state_type_]->callCancel(req, res);
}

bool Server::floatCB(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_srvs::Float::Response& res)
{
  return state_[state_type_]->callFloat(req, res);
}

bool Server::setJointCB(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_srvs::SetJoint::Response& res)
{
  return state_[state_type_]->callSetJoint(req, res);
}

bool Server::jointSpaceControlCB(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_srvs::JointSpaceControl::Response& res)
{
  return state_[state_type_]->callJointSpaceControl(req, res);
}

bool Server::taskSpaceControlCB(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_srvs::TaskSpaceControl::Response& res)
{
  return state_[state_type_]->callTaskSpaceControl(req, res);
}

bool Server::taskSpaceHybridControlCB(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_srvs::TaskSpaceHybridControl::Response& res)
{
  return state_[state_type_]->callTaskSpaceHybridControl(req, res);
}
