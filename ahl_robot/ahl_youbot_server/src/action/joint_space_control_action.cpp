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

#include <ahl_robot_controller/task/gravity_compensation.hpp>
#include <ahl_robot_controller/task/joint_control.hpp>
#include "ahl_youbot_server/action/joint_space_control_action.hpp"

using namespace ahl_youbot;

JointSpaceControlAction::JointSpaceControlAction(const std::string& action_name, const ahl_robot::RobotPtr& robot, const ahl_ctrl::RobotControllerPtr& controller, const ahl_youbot::InterfacePtr& interface)
  : Action(action_name), robot_(robot), controller_(controller), interface_(interface)
{
  req_ = JointSpaceRequestPtr(new JointSpaceRequest());

  task_[GRAVITY_COMPENSATION] = ahl_ctrl::TaskPtr(
    new ahl_ctrl::GravityCompensation(robot_->getManipulator("mnp")));
  task_[JOINT_CONTROL] = ahl_ctrl::TaskPtr(
    new ahl_ctrl::JointControl(robot_->getManipulator("mnp")));
}

void JointSpaceControlAction::init()
{
  controller_->clearTask();

  std::map<JointSpaceControlAction::TaskList, ahl_ctrl::TaskPtr>::iterator it;
  for(it = task_.begin(); it != task_.end(); ++it)
  {
    const int priority = 0;
    controller_->addTask(it->second, priority);
  }
}

void JointSpaceControlAction::execute(void* goal)
{
  Eigen::VectorXd q = Eigen::VectorXd::Constant(robot_->getDOF("mnp"), M_PI / 4.0);
  task_[JOINT_CONTROL]->setGoal(q);

  Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot_->getDOF("mnp"));
  controller_->computeGeneralizedForce(tau);
  interface_->applyJointEfforts(tau);
}
