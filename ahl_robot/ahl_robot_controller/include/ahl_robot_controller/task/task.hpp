#ifndef __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP
#define __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/param.hpp"

namespace ahl_ctrl
{
  class Task
  {
  public:
    virtual ~Task() {}
    virtual void setParam(const ParamPtr& param) { param_ = param; }
    virtual void setGoal(const Eigen::MatrixXd& dst) {}
    virtual void updateModel() {}
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau) {}
    virtual bool haveNullSpace() { return false; }
    const Eigen::MatrixXd getNullSpace() const { return N_; }

  protected:
    ahl_robot::ManipulatorPtr mnp_;
    Eigen::MatrixXd N_;
    ParamPtr param_;
  };

  typedef boost::shared_ptr<Task> TaskPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_HPP_TASK_HPP */
