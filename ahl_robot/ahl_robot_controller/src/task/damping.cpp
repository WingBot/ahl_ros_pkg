#include "ahl_robot_controller/task/damping.hpp"

using namespace ahl_ctrl;

Damping::Damping(const ahl_robot::ManipulatorPtr& mnp)
{
  mnp_ = mnp;
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void Damping::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = -param_->getKvDamp() * mnp_->dq;
}
