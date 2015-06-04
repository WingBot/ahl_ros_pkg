#include <ahl_robot/robot/manipulator.hpp>
#include <ahl_robot_controller/task/joint_control.hpp>
#include <ahl_robot_controller/task/gravity_compensation.hpp>
#include "ahl_youbot_server/action/float_action.hpp"

using namespace ahl_youbot;
using namespace ahl_robot;

FloatAction::FloatAction(const std::string& action_name, const ahl_robot::RobotPtr& robot, const ahl_ctrl::RobotControllerPtr& controller, const ahl_youbot::InterfacePtr& interface)
  : Action(action_name), robot_(robot)
{
  req_ = FloatRequestPtr(new FloatRequest());
  controller_ = controller;
  //task_ = ahl_ctrl::TaskPtr(new ahl_ctrl::GravityCompensation(robot->getManipulator("mnp"), 0));
  task_ = ahl_ctrl::TaskPtr(new ahl_ctrl::JointControl(robot->getManipulator("mnp"), 0));
  controller_->addTask(task_);
  interface_ = interface;
}

void FloatAction::execute(void* goal)
{
  ManipulatorPtr mnp = robot_->getManipulator("mnp");
  task_->setGoal(mnp->q);

  //std::cout << "q : " << std::endl << mnp->q << std::endl
  //          << "dq : " << std::endl << mnp->dq << std::endl;

  Eigen::VectorXd tau(mnp->dof);
  controller_->computeGeneralizedForce(tau);

  Eigen::VectorXd tau_arm(mnp->dof - 3);
  for(unsigned int i = 0; i < tau_arm.rows(); ++i)
  {
    tau_arm.coeffRef(i) = tau.coeff(i + 3);
  }

  interface_->applyJointEfforts(tau_arm);
  //ROS_INFO_STREAM("FloatAction");
  //std::cout << tau_arm << std::endl << std::endl;
}
