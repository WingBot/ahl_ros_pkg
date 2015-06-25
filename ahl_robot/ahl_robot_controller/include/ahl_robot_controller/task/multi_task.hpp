#ifndef __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP
#define __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class MultiTask
  {
  public:
    void addTask(const TaskPtr& task, int priority);
    void clear();
    void updateModel();
    void computeGeneralizedForce(int dof, Eigen::VectorXd& tau);

  private:
    std::map<int, std::vector<TaskPtr> > multi_task_; // key : priority
    Eigen::MatrixXd N_;
  };

  typedef boost::shared_ptr<MultiTask> MultiTaskPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_MULTI_TASK_HPP */
