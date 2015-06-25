#ifndef __AHL_ROBOT_CONTROLLER_DAMPING_HPP
#define __AHL_ROBOT_CONTROLLER_DAMPING_HPP

#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class Damping : public Task
  {
  public:
    Damping(const ahl_robot::ManipulatorPtr& mnp);
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_DAMPING_HPP */
