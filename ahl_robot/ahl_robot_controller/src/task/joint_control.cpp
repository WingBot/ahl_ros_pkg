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
  double kp = 500.0;
  double kv = 50.0;

  Eigen::VectorXd tau_unit = -kp * (mnp_->q - qd_) - kv * mnp_->dq;
  tau = mnp_->M * tau_unit;
}
