#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/friction_compensation.hpp"

using namespace ahl_ctrl;

FrictionCompensation::FrictionCompensation(const ahl_robot::RobotPtr& robot)
{
  robot_ = robot;
  mnp_name_ = robot_->getManipulatorName();
  N_ = Eigen::MatrixXd::Identity(robot->getDOF(), robot->getDOF());
  b_ = Eigen::VectorXd::Zero(robot->getDOF()).asDiagonal();
}

void FrictionCompensation::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(robot_->getDOF());
  unsigned int macro_dof = robot_->getMacroManipulatorDOF();

  unsigned int offset = 0;
  for(unsigned int i = 0; i < mnp_name_.size(); ++i)
  {
    mnp_ = robot_->getManipulator(mnp_name_[i]);
    unsigned int mini_dof = mnp_->dof - macro_dof;

    tau.block(macro_dof + offset, 0, mini_dof, 1) = b_.block(macro_dof + offset, macro_dof + offset, mini_dof, mini_dof) * mnp_->dq.block(macro_dof, 0, mini_dof, 1);

    offset += mini_dof;
  }

  tau_ = tau;
}
