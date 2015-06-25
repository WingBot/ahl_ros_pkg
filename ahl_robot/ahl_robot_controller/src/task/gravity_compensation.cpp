#include "ahl_robot_controller/task/gravity_compensation.hpp"

using namespace ahl_ctrl;

GravityCompensation::GravityCompensation(const ahl_robot::ManipulatorPtr& mnp)
{
  mnp_ = mnp;
  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
  g_ << 0.0, 0.0, -0.980;
}

GravityCompensation::GravityCompensation(const ahl_robot::ManipulatorPtr& mnp, const Eigen::Vector3d& g)
{
  mnp_ = mnp;
  g_ = g;
}

void GravityCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->dof);
  for(unsigned int i = 0; i < mnp_->link.size(); ++i)
  {
    tau -= mnp_->link[i]->m * mnp_->J0[i].block(0, 0, 3, mnp_->J0[i].cols()).transpose() * g_;
  }
}
