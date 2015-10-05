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

#include <cmath>
#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/hybrid_control.hpp"
#include "ahl_robot_controller/common/effective_mass_matrix3d.hpp"

using namespace ahl_ctrl;

HybridControl::HybridControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, const Eigen::Matrix3d& Rf, const Eigen::Matrix3d& Rm, double zero_thresh, double eigen_thresh)
  : updated_(false),
    Rf_(Rf),
    Rm_(Rm),
    zero_thresh_(zero_thresh)
{
  mnp_ = mnp;

  tasks_[PositionControl] = TaskPtr(
    new ahl_ctrl::PositionControl(mnp, target_link, eigen_thresh));
  tasks_[OrientationControl] = TaskPtr(
    new ahl_ctrl::OrientationControl(mnp, target_link, eigen_thresh));

  idx_ = mnp_->name_to_idx[target_link];

  I3_ = Eigen::Matrix3d::Identity();
  N_  = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);

  fd_ = Eigen::VectorXd::Zero(6);

  sigma_f_ = I3_;
  sigma_m_ = I3_;
  sigma_f_bar_ = I3_ - sigma_f_;
  sigma_m_bar_ = I3_ - sigma_m_;

  omega_ = Eigen::MatrixXd::Zero(sigma_f_.rows() + sigma_m_.rows(), sigma_f_.cols() + sigma_m_.cols());
  omega_.block(0, 0, 3, 3) = Rf_.transpose() * sigma_f_ * Rf_;
  omega_.block(3, 3, 3, 3) = Rm_.transpose() * sigma_m_ * Rm_;
  omega_bar_.block(0, 0, 3, 3) = Rf_.transpose() * sigma_f_bar_ * Rf_;
  omega_bar_.block(3, 3, 3, 3) = Rm_.transpose() * sigma_m_bar_ * Rm_;
}

void HybridControl::setGoal(const Eigen::MatrixXd& ref)
{
  if(ref.rows() != 12)
  {
    std::stringstream msg;

    msg << "ref.rows() != 12" << std::endl
        << "  ref.rows : " << ref.rows();

    throw ahl_ctrl::Exception("HybridControl::setGoal", msg.str());
  }

  tasks_[PositionControl]->setGoal(ref.block(0, 0, 3, 1));

  Eigen::Matrix3d Rd;
  Rd = Eigen::AngleAxisd(ref.coeff(5, 0), Eigen::Vector3d::UnitX())
     * Eigen::AngleAxisd(ref.coeff(4, 0), Eigen::Vector3d::UnitY())
     * Eigen::AngleAxisd(ref.coeff(3, 0), Eigen::Vector3d::UnitZ());
  tasks_[OrientationControl]->setGoal(Rd);

  fd_ = ref.block(6, 0, 6, 1);

  sigma_f_ = I3_;
  sigma_m_ = I3_;

  for(unsigned int i = 0; i < 3; ++i)
  {
    if(fabs(fd_[i]) > zero_thresh_)
    {
      sigma_f_.coeffRef(i, i) = 0.0;
    }
  }

  for(unsigned int i = 3; i < 6; ++i)
  {
    if(fabs(fd_[i]) > zero_thresh_)
    {
      sigma_m_.coeffRef(i, i) = 0.0;
    }
  }

  sigma_f_bar_ = I3_ - sigma_f_;
  sigma_m_bar_ = I3_ - sigma_m_;

  omega_.block(0, 0, 3, 3) = Rf_.transpose() * sigma_f_ * Rf_;
  omega_.block(3, 3, 3, 3) = Rm_.transpose() * sigma_m_ * Rm_;
  omega_bar_.block(0, 0, 3, 3) = Rf_.transpose() * sigma_f_bar_ * Rf_;
  omega_bar_.block(3, 3, 3, 3) = Rm_.transpose() * sigma_m_bar_ * Rm_;
}

void HybridControl::updateModel()
{
  tasks_[PositionControl]->updateModel();
  tasks_[OrientationControl]->updateModel();

  N_ = tasks_[PositionControl]->getNullSpace()
     * tasks_[OrientationControl]->getNullSpace();

  // update model for force control
  updated_ = true;
}

void HybridControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  Eigen::VectorXd tau_p;
  Eigen::VectorXd tau_o;

  tasks_[PositionControl]->computeGeneralizedForce(tau_p);
  tasks_[OrientationControl]->computeGeneralizedForce(tau_o);

  tau = tau_p + tau_o;
}
