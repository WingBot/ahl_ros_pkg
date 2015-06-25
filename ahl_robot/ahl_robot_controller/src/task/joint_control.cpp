#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/joint_control.hpp"

using namespace ahl_ctrl;

JointControl::JointControl(const ahl_robot::ManipulatorPtr& mnp)
{
  mnp_ = mnp;
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void JointControl::setGoal(const Eigen::MatrixXd& qd)
{
  if(qd.rows() != mnp_->dof)
  {
    std::stringstream msg;
    msg << "qd.rows() != mnp_->dof" << std::endl
        << "  qd.rows   : " << qd.rows() << std::endl
        << "  mnp_->dof : " << mnp_->dof;
    throw ahl_ctrl::Exception("JointControl::setGoal", msg.str());
  }

  qd_ = qd.block(0, 0, qd.rows(), 1);
}

void JointControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  double kp = 1.0;
  double kv = 0.12;

  Eigen::VectorXd tau_unit = -kp * (mnp_->q - qd_) - kv * mnp_->dq;
  tau = mnp_->M * tau_unit;
}
