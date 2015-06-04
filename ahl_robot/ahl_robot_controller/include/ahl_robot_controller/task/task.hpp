#ifndef __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP
#define __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ahl_robot/ahl_robot.hpp>

namespace ahl_ctrl
{
  class Task
  {
  public:
    virtual ~Task() {}
    virtual void setGoal(const Eigen::VectorXd& dst) {}
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau) {}
    virtual int getPriority()
    {
      return priority_;
    }

  protected:
    ahl_robot::ManipulatorPtr mnp_;
    int priority_;
  };

  typedef boost::shared_ptr<Task> TaskPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP */
