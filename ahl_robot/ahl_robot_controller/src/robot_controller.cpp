#include "ahl_robot_controller/robot_controller.hpp"

using namespace ahl_ctrl;

RobotController::RobotController()
{
  
}

void RobotController::init(const ahl_robot::RobotPtr& robot, const std::string& mnp_name)
{
  mnp_ = robot->getManipulator(mnp_name);
}

void RobotController::addTask(const TaskPtr& task)
{
  std::list<TaskPtr>::iterator it;
  for(it = task_.begin(); it != task_.end(); ++it)
  {
    if(task->getPriority() > (*it)->getPriority())
    {
      task_.insert(it, task);
      return;
    }
  }
  task_.push_back(task);
}

void RobotController::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->dof);

  std::list<TaskPtr>::iterator it;
  for(it = task_.begin(); it != task_.end(); ++it)
  {
    Eigen::VectorXd tau_task;
    (*it)->computeGeneralizedForce(tau_task);
    tau += tau_task;
  }
}
