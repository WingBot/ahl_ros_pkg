#include "ahl_robot_controller/task/damping.hpp"

using namespace ahl_ctrl;

Damping::Damping(const ahl_robot::ManipulatorPtr& mnp)
{
  mnp_ = mnp;
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void Damping::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  double kp = 10.0;
  double kv = 1.0;

  tau = -kv * mnp_->dq;
}
