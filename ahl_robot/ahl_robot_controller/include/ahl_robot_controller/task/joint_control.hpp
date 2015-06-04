#ifndef __AHL_ROBOT_CONTROLLER_JOINT_CONTROL_HPP
#define __AHL_ROBOT_CONTROLLER_JOINT_CONTROL_HPP

#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class JointControl : public Task
  {
  public:
    JointControl(const ahl_robot::ManipulatorPtr& mnp, int priority);
    void setGoal(const Eigen::VectorXd& qd)
    {
      qd_ = qd;
    }
    void computeGeneralizedForce(Eigen::VectorXd& tau);

  private:
    Eigen::VectorXd qd_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_JOINT_CONTROL_HPP */
