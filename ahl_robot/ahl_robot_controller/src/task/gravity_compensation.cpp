#include "ahl_robot_controller/task/gravity_compensation.hpp"

using namespace ahl_ctrl;

GravityCompensation::GravityCompensation(const ahl_robot::ManipulatorPtr& mnp, int priority)
{
  mnp_ = mnp;
  priority_ = priority;
}

void GravityCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  Eigen::Vector3d g;
  g << 0.0, 0.0, -9.8;

  tau = Eigen::VectorXd::Zero(mnp_->dof);
  for(unsigned int i = 0; i < mnp_->link.size(); ++i)
  {
    //std::cout << mnp_->J0.size() << ", " << mnp_->link.size() << std::endl;
    tau += -mnp_->link[i]->m * mnp_->J0[i].block(0, 0, 3, mnp_->J0[i].cols()).transpose() * g;
  }
}
