#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/task/multi_task.hpp"

using namespace ahl_ctrl;

void MultiTask::addTask(const TaskPtr& task, int priority)
{
  if(multi_task_.find(priority) != multi_task_.end())
  {
    if(task->haveNullSpace())
    {
      for(unsigned int i = 0; i < multi_task_[priority].size(); ++i)
      {
        if(multi_task_[priority][i]->haveNullSpace())
        {
          std::stringstream msg;
          msg << "Two tasks with the same priority have null spaces." << std::endl
              << "  priority : " << priority;
          throw ahl_ctrl::Exception("MultiTask::addTask", msg.str());
        }
      }
    }
  }

  multi_task_[priority].push_back(task);
}

void MultiTask::clear()
{
  multi_task_.clear();
}

void MultiTask::updateModel()
{
  std::map<int, std::vector<TaskPtr> >::iterator it;
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    for(unsigned int i = 0; i < it->second.size(); ++i)
    {
      it->second[i]->updateModel();
    }
  }
}

void MultiTask::computeGeneralizedForce(int dof, Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(dof);

  std::map<int, std::vector<TaskPtr> >::iterator it = multi_task_.begin();
  for(it = multi_task_.begin(); it != multi_task_.end(); ++it)
  {
    N_ = Eigen::MatrixXd::Identity(dof, dof);
    Eigen::VectorXd tau_sum = Eigen::VectorXd::Zero(dof);

    for(unsigned int i = 0; i < it->second.size(); ++i)
    {
      Eigen::VectorXd tau_task;
      it->second[i]->computeGeneralizedForce(tau_task);
      tau_sum += tau_task;

      if(it->second[i]->haveNullSpace())
      {
        N_ = it->second[i]->getNullSpace();
      }
    }

    tau = N_ * tau;
    tau += tau_sum;
  }
}

