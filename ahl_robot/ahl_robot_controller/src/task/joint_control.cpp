#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/joint_control.hpp"

using namespace ahl_ctrl;

JointControl::JointControl(const ahl_robot::ManipulatorPtr& mnp, int priority)
{
  mnp_ = mnp;
  priority_ = priority;
}

void JointControl::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  double kp = 60.0;
  double kv = 5.0;

  Eigen::VectorXd tau_unit = -kp * (mnp_->q - qd_) - kv * mnp_->dq;
  tau = mnp_->M * tau_unit;

  //tau.coeffRef(tau.rows() - 1) = 0.005 * tau_unit.coeff(tau_unit.rows() - 1);

  //std::cout << "J : " << std::endl << mnp_->J0[mnp_->J0.size() - 1] << std::endl << std::endl;
  //std::cout << "M : " << std::endl << mnp_->M << std::endl << std::endl;
  //std::cout << "tau : " << std::endl << tau << std::endl << std::endl;
  //std::cout << mnp_->q * (1.0 / M_PI) * 180.0 << std::endl << std::endl;
  //std::cout << qd_ * (1.0 / M_PI) * 180.0 << std::endl << std::endl;
  //std::cout << tau.coeff(3) << std::endl << std::endl;

  //tau.coeffRef(3) = 30.0;

  Eigen::Vector3d g;
  g << 0.0, 0.0, -9.80665;
  Eigen::VectorXd tau_gravity = Eigen::VectorXd::Zero(mnp_->dof);
  for(unsigned int i = 0; i < mnp_->link.size(); ++i)
  {
    tau_gravity -= mnp_->link[i]->m * mnp_->J0[i].block(0, 0, 3, mnp_->J0[i].cols()).transpose() * g;
  }

  tau += tau_gravity;
}
