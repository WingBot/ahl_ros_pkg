#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/common/effective_mass_matrix3d.hpp"
#include "ahl_robot_controller/task/orientation_control.hpp"

using namespace ahl_ctrl;

OrientationControl::OrientationControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh)
  : updated_(false), target_link_(target_link), eigen_thresh_(eigen_thresh)
{
  mnp_ = mnp;

  if(mnp_->name_to_idx.find(target_link) == mnp_->name_to_idx.end())
  {
    std::stringstream msg;
    msg << target_link << " was not found in mnp_->name_to_idx.";
    throw ahl_ctrl::Exception("OrientationControl::OrientationControl", msg.str());
  }

  idx_ = mnp_->name_to_idx[target_link];
  I_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void OrientationControl::setGoal(const Eigen::MatrixXd& Rd)
{
  if(Rd.rows() != 3)
  {
    std::stringstream msg;
    msg << "Rd.rows() != 3" << std::endl
        << "  Rd.rows : " << Rd.rows();
    throw ahl_ctrl::Exception("OrientationControl::setGoal", msg.str());
  }
  if(Rd.cols() != 3)
  {
    std::stringstream msg;
    msg << "Rd.cols() != 3" << std::endl
        << "  Rd.cols : " << Rd.cols();
    throw ahl_ctrl::Exception("OrientationControl::setGoal", msg.str());
  }

  Rd_ = Rd;
}

void OrientationControl::updateModel()
{
  Jw_ = mnp_->J0[idx_].block(3, 0, 3, mnp_->J0[idx_].cols());
  lambda_inv_ = Jw_ * mnp_->M_inv * Jw_.transpose();
  EffectiveMassMatrix3d::compute(lambda_inv_, lambda_, eigen_thresh_);
  J_dyn_inv_ = mnp_->M_inv * Jw_.transpose() * lambda_;
  N_ = I_ - Jw_.transpose() * J_dyn_inv_.transpose();

  updated_ = true;
}

void OrientationControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  if(!updated_)
  {
    tau = Eigen::VectorXd::Zero(mnp_->dof);
    return;
  }

  double kp = 20.0;
  double kv = 2.0;

  Eigen::Matrix3d R = mnp_->T_abs[idx_].block(0, 0, 3, 3);
  Eigen::Quaternion<double> q;
  q = R * Rd_.inverse();
  double norm = sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z());
  Eigen::Vector3d del_phi;
  double c = 0.0;
  if(norm != 0.0)
  {
    c = 1.0 / norm * 2.0 * acos(q.w());
  }
  del_phi << q.x() * c, q.y() * c, q.z() * c;

  Eigen::VectorXd M_unit = -kp * del_phi -kv * Jw_ * mnp_->dq;
  Eigen::VectorXd M = lambda_ * M_unit;
  tau = Jw_.transpose() * M;
}

