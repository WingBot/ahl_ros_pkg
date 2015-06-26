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

#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/robot.hpp"

using namespace ahl_robot;

void Robot::update(const std::string& mnp_name, const Eigen::VectorXd& q)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::update", msg.str());
  }

  mnp_[mnp_name]->update(q);
}

void Robot::computeBasicJacobian(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::BasicJacobian", msg.str());
  }

  mnp_[mnp_name]->computeBasicJacobian();
}

void Robot::computeMassMatrix(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::computeMassMatrix", msg.str());
  }

  mnp_[mnp_name]->computeMassMatrix();
}

bool Robot::reached(const std::string& mnp_name, const Eigen::VectorXd& qd, double threshold)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::reached", msg.str());
  }

  return mnp_[mnp_name]->reached(qd, threshold);
}

void Robot::add(const ManipulatorPtr& mnp)
{
  mnp_[mnp->name] = mnp;
  mnp_name_.push_back(mnp->name);
}

const Eigen::MatrixXd& Robot::getBasicJacobian(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  std::string link_name = mnp_[mnp_name]->link[mnp_[mnp_name]->link.size() - 1]->name;
  return this->getBasicJacobian(mnp_name, link_name);
}

const Eigen::MatrixXd& Robot::getBasicJacobian(const std::string& mnp_name, const std::string& link_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  if(mnp_[mnp_name]->name_to_idx.find(link_name) == mnp_[mnp_name]->name_to_idx.end())
  {
    std::stringstream msg;
    msg << "Could not find name_to_idx." << std::endl
        << "  Manipulator : " << mnp_name << std::endl
        << "  Link        : " << link_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  int idx = mnp_[mnp_name]->name_to_idx[link_name];
  return mnp_[mnp_name]->J0[idx];
}

const Eigen::MatrixXd& Robot::getMassMatrix(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getMassMatrix", msg.str());
  }

  return mnp_[mnp_name]->M;
}

const Eigen::MatrixXd& Robot::getMassMatrixInv(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getMassMatrixInv", msg.str());
  }

  return mnp_[mnp_name]->M_inv;
}

const Eigen::VectorXd& Robot::getJointPosition(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getJointPosition", msg.str());
  }

  return mnp_[mnp_name]->q;
}

const Eigen::VectorXd& Robot::getJointVelocity(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getJointVelocity", msg.str());
  }

  return mnp_[mnp_name]->dq;
}

unsigned int Robot::getDOF(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getDOF", msg.str());
  }

  return mnp_[mnp_name]->dof;
}
