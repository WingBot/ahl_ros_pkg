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
  Eigen::VectorXd tau_unit = -param_->getKp() * (mnp_->q - qd_) - param_->getKv() * mnp_->dq;
  tau = mnp_->M * tau_unit;
  Eigen::FullPivLU<Eigen::MatrixXd> lu(mnp_->M);

  std::cout << "q" << std::endl << mnp_->q << std::endl << std::endl;
  std::cout << "qd" << std::endl << qd_ << std::endl << std::endl;
  std::cout << "rank" << std::endl << lu.rank() << std::endl << std::endl;
}
