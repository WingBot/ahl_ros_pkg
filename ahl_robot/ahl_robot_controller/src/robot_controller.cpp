#include "ahl_robot_controller/robot_controller.hpp"

using namespace ahl_ctrl;

RobotController::RobotController()
{
  multi_task_ = MultiTaskPtr(new MultiTask());  
}

void RobotController::init(const ahl_robot::RobotPtr& robot, const std::string& mnp_name)
{
  mnp_ = robot->getManipulator(mnp_name);
}

void RobotController::addTask(const TaskPtr& task, int priority)
{
  multi_task_->addTask(task, priority);
}

void RobotController::clearTask()
{
  multi_task_->clear();
}

void RobotController::updateModel()
{
  multi_task_->updateModel();
}

void RobotController::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  multi_task_->computeGeneralizedForce(mnp_->dof, tau);
}
