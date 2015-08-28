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

#include "ahl_youbot_sample/viewer/viewer.hpp"
#include "ahl_youbot_sample/exception.hpp"

using namespace ahl_youbot;

Viewer::Viewer()
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("ahl_youbot_sample");

  double duration;
  std::string cfg_youbot_base;
  std::string cfg_youbot_manipulator;

  local_nh.param<double>("tele_operation/duration", duration, 0.5);
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_base", cfg_youbot_base, "youbot-base");
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_manipulator", cfg_youbot_manipulator, "youbot-manipulator");

  timer_ = nh.createTimer(ros::Duration(duration), &Viewer::timerCB, this);

  base_ = YouBotBasePtr(
    new youbot::YouBotBase(cfg_youbot_base, YOUBOT_CONFIGURATIONS_DIR));
  base_->doJointCommutation();

  manipulator_ = YouBotManipulatorPtr(
    new youbot::YouBotManipulator(cfg_youbot_manipulator, YOUBOT_CONFIGURATIONS_DIR));
  manipulator_->doJointCommutation();
}

void Viewer::timerCB(const ros::TimerEvent&)
{
  base_->getBaseVelocity(vx_, vy_, vr_);
  manipulator_->getJointData(q_);
  manipulator_->getJointData(dq_);
  manipulator_->getJointData(tau_);

  std::cout << "Base : " << std::endl
            << "(vx, vy, vr) = ("
            << vx_.value() << ", "
            << vy_.value() << ", "
            << vr_.value() << ")" << std::endl;

  const int arm_dof = 5;

  if(q_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of q is wrong. It should be 5." << std::endl
        << "  size : " << q_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }
  else if(dq_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of dq is wrong. It should be 5." << std::endl
        << "  size : " << dq_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }
  else if(tau_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of tau is wrong. It should be 5." << std::endl
        << "  size : " << tau_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }

  std::cout << "Manipulator : " << std::endl
            << "(q1, q2, q3, q4, q5) = ("
            << q_[0].angle.value() << ", "
            << q_[1].angle.value() << ", "
            << q_[2].angle.value() << ", "
            << q_[3].angle.value() << ", "
            << q_[4].angle.value() << ")" << std::endl
            << "(dq1, dq2, dq3, dq4, dq5) = ("
            << dq_[0].angularVelocity.value() << ", "
            << dq_[1].angularVelocity.value() << ", "
            << dq_[2].angularVelocity.value() << ", "
            << dq_[3].angularVelocity.value() << ", "
            << dq_[4].angularVelocity.value() << ")" << std::endl
            << "(tau1, tau2, tau3, tau4, tau5) = ("
            << tau_[0].torque.value() << ", "
            << tau_[1].torque.value() << ", "
            << tau_[2].torque.value() << ", "
            << tau_[3].torque.value() << ", "
            << tau_[4].torque.value() << ")" << std::endl;
}
