#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/common/effective_mass_matrix3d.hpp"
#include "ahl_robot_controller/task/position_control.hpp"

using namespace ahl_ctrl;

PositionControl::PositionControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
  mnp_ = mnp;

  if(mnp_->name_to_idx.find(target_link) == mnp_->name_to_idx.end())
  {
    std::stringstream msg;
    msg << target_link << " was not found in mnp_->name_to_idx.";
    throw ahl_ctrl::Exception("PositionControl::PositionControl", msg.str()); 
  }

  idx_ = mnp_->name_to_idx[target_link];
  I_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void PositionControl::setGoal(const Eigen::MatrixXd& xd)
{
  if(xd.rows() != 3)
  {
    std::stringstream msg;
    msg << "xd.rows() != 3" << std::endl
        << "  xd.rows : " << xd.rows();
    throw ahl_ctrl::Exception("PositionControl::setGoal", msg.str());
  }

  xd_ = xd.block(0, 0, xd.rows(), 1);
}

void PositionControl::updateModel()
{
  Jv_ = mnp_->J0[idx_].block(0, 0, 3, mnp_->J0[idx_].cols());
  lambda_inv_ = Jv_ * mnp_->M_inv * Jv_.transpose();
  EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
  J_dyn_inv_ = mnp_->M_inv * Jv_.transpose() * lambda_;
  N_ = I_ - Jv_.transpose() * J_dyn_inv_.transpose();
  updated_ = true;
}

void PositionControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  if(!updated_)
  {
    tau = Eigen::VectorXd::Zero(mnp_->dof);
    return;
  }

  double kp = 10.0;
  double kv = 0.5;

  Eigen::Vector3d x = mnp_->T_abs[idx_].block(0, 3, 3, 1);
  Eigen::VectorXd F_unit = -kp * (x - xd_) - kv * Jv_ * mnp_->dq;
  Eigen::VectorXd F = lambda_ * F_unit;

  tau = Jv_.transpose() * F;

  std::cout << "xp" << std::endl << mnp_->T_abs[mnp_->T_abs.size() - 1].block(0, 3, 3, 1) << std::endl;
}
