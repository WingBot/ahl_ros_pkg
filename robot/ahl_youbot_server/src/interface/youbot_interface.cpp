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

#include "ahl_youbot_server/exception.hpp"
#include "ahl_youbot_server/interface/youbot_interface.hpp"

using namespace ahl_youbot;

YoubotInterface::YoubotInterface(unsigned int dof, const std::string& ahl_cfg_yaml, const std::string& cfg_path, bool do_calibration)
  : dof_(dof), base_dof_(3)
{
  base_ = YouBotBasePtr(
    new youbot::YouBotBase(youbot_if::CONFIG_BASE, cfg_path));
  mnp_ = YouBotManipulatorPtr(
    new youbot::YouBotManipulator(youbot_if::CONFIG_MANIPULATOR, cfg_path));

  arm_dof_ = dof_ - base_dof_;
  q_offset_ = Eigen::VectorXd::Zero(arm_dof_);
  flip_direction_ = Eigen::VectorXi::Zero(arm_dof_);

  base_->doJointCommutation();
  mnp_->doJointCommutation();
  mnp_->calibrateManipulator(do_calibration);

  this->init(ahl_cfg_yaml);
}

YoubotInterface::~YoubotInterface()
{
  std::vector<youbot::JointTorqueSetpoint> tau_arm(arm_dof_);

  for(unsigned int i = 0; i < arm_dof_; ++i)
  {
    tau_arm[i].torque = 0.0 * newton_meter;
    mnp_->setJointData(tau_arm);
  }

  youbot::PParameterCurrentControl p_param;
  youbot::IParameterCurrentControl i_param;
  youbot::DParameterCurrentControl d_param;

  p_param.setParameter(25); // Default value
  i_param.setParameter(60); // Default value
  d_param.setParameter(0);  // Default value

  for(unsigned int i = 0; i < arm_dof_; ++i)
  {
    mnp_->getArmJoint(i + 1).setConfigurationParameter(p_param);
    mnp_->getArmJoint(i + 1).setConfigurationParameter(i_param);
    mnp_->getArmJoint(i + 1).setConfigurationParameter(d_param);
  }
}

bool YoubotInterface::getJointStates(Eigen::VectorXd& q)
{
  std::vector<youbot::JointSensedAngle> q_msr;
  mnp_->getJointData(q_msr);

  for(unsigned int i = 0; i < q_msr.size(); ++i)
  {
    double tmp = q_msr[i].angle.value();
    if(flip_direction_[i])
    {
      tmp *= -1.0;
    }

    q[i + base_dof_] = tmp + q_offset_[i];
  }

  return true;
}

bool YoubotInterface::getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq)
{
  // TODO implement
  return true;
}

// Assume tau size includes mobility
bool YoubotInterface::getJointEfforts(Eigen::VectorXd& tau)
{
  std::vector<youbot::JointSensedTorque> tau_msr;
  mnp_->getJointData(tau_msr);

  for(unsigned int i = 0; i < tau_msr.size(); ++i)
  {
    double tmp = tau_msr[i].torque.value();
    if(flip_direction_[i])
    {
      tmp *= -1.0;
    }

    tau[i + base_dof_] = tmp;
  }

  return true;
}

bool YoubotInterface::getOdometry(Eigen::Vector3d& q)
{
  quantity<si::length> x;
  quantity<si::length> y;
  quantity<si::plane_angle> yaw;

  base_->getBasePosition(x, y, yaw);

  q[0] = x.value();
  q[1] = y.value();
  q[2] = yaw.value();

  return true;
}

bool YoubotInterface::getOdometry(Eigen::Vector3d& q, Eigen::Vector3d& dq)
{
  // TODO implement
  return true;
}

bool YoubotInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  std::vector<youbot::JointTorqueSetpoint> tau_arm(arm_dof_);

  // Be careful about the direction
  for(unsigned int i = 0; i < arm_dof_; ++i)
  {
    if(flip_direction_[i])
    {
      tau_arm[i].torque = -tau[i + base_dof_] * newton_meter;
    }
    else
    {
      tau_arm[i].torque = tau[i + base_dof_] * newton_meter;
    }

    mnp_->setJointData(tau_arm);
  }

  return true;
}

bool YoubotInterface::applyBaseVelocity(const Eigen::Vector3d& v)
{
  return true;
}

void YoubotInterface::init(const std::string& ahl_cfg_yaml)
{
  std::ifstream ifs(ahl_cfg_yaml.c_str());

  if(!ifs)
  {
    std::stringstream msg;
    msg << "Could not find \"" << ahl_cfg_yaml << "\".";
    throw ahl_youbot::Exception("YoubotInterface::init", msg.str());
  }

  YAML::Node node = YAML::Load(ifs);

  youbot::PParameterCurrentControl p_param;
  youbot::IParameterCurrentControl i_param;
  youbot::DParameterCurrentControl d_param;

  for(unsigned int i = 0; i < arm_dof_; ++i)
  {
    std::stringstream tag;
    tag << youbot_if::tag::JOINT << i + 1;
    this->checkTag(node, tag.str());

    YAML::Node node_joint = node[tag.str()];

    this->checkTag(node_joint, youbot_if::tag::OFFSET);
    this->checkTag(node_joint, youbot_if::tag::FLIP_DIRECTION);
    this->checkTag(node_joint, youbot_if::tag::P_CURRENT);
    this->checkTag(node_joint, youbot_if::tag::I_CURRENT);
    this->checkTag(node_joint, youbot_if::tag::D_CURRENT);

    q_offset_[i] = node_joint[youbot_if::tag::OFFSET].as<double>();
    flip_direction_[i] = node_joint[youbot_if::tag::FLIP_DIRECTION].as<int>();

    p_param.setParameter(node_joint[youbot_if::tag::P_CURRENT].as<double>());
    i_param.setParameter(node_joint[youbot_if::tag::I_CURRENT].as<double>());
    d_param.setParameter(node_joint[youbot_if::tag::D_CURRENT].as<double>());

    mnp_->getArmJoint(i + 1).setConfigurationParameter(p_param);
    mnp_->getArmJoint(i + 1).setConfigurationParameter(i_param);
    mnp_->getArmJoint(i + 1).setConfigurationParameter(d_param);
  }
}

void YoubotInterface::checkTag(const YAML::Node& node, const std::string& tag)
{
  if(!node[tag])
  {
    std::stringstream msg;
    msg << "Could not find tag : " << tag;
    throw ahl_youbot::Exception("YoubotInterface::checkTag", msg.str());
  }
}
