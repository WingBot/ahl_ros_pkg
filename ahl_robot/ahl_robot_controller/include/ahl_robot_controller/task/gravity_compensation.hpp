#ifndef __AHL_ROBOT_CONTROLLER_GRAVITY_COMPENSATION_HPP
#define __AHL_ROBOT_CONTROLLER_GRAVITY_COMPENSATION_HPP

#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class GravityCompensation : public Task
  {
  public:
    GravityCompensation(const ahl_robot::ManipulatorPtr& mnp);
    GravityCompensation(const ahl_robot::ManipulatorPtr& mnp, const Eigen::Vector3d& g);
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
  private:
    Eigen::Vector3d g_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_GRAVITY_COMPENSATION_HPP */
