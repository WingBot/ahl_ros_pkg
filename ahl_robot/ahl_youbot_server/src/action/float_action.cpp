#include <ros/ros.h>
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
  Eigen::VectorXd q = Eigen::VectorXd::Zero(mnp->dof);
  double sinval = sin(2.0 * M_PI * 0.1 * ros::Time::now().toNSec() * 0.001 * 0.001 * 0.001);

  q.coeffRef(0) = mnp->q.coeff(0);
  q.coeffRef(1) = mnp->q.coeff(1);
  q.coeffRef(2) = mnp->q.coeff(2);

  q.coeffRef(3) =  M_PI / 4.0 * sinval;
  q.coeffRef(4) =  M_PI / 4.0 * sinval;
  q.coeffRef(5) =  M_PI / 4.0 * sinval;
  q.coeffRef(6) =  M_PI / 4.0 * sinval;
  q.coeffRef(7) =  M_PI / 4.0 * sinval;

  std::cout << "diff : " << std::endl << (q.coeff(7) - mnp->q.coeff(7)) * (1.0 / M_PI) * 180.0 << std::endl << std::endl;

  task_->setGoal(q);

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
}
