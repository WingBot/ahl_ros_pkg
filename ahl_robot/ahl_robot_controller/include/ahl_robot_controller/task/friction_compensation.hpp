#ifndef __AHL_ROBOT_CONTROLLER_FRICTION_COMPENSATION_HPP
#define __AHL_ROBOT_CONTROLLER_FRICTION_COMPENSATION_HPP

#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class FrictionCompensation : public Task
  {
  public:
    FrictionCompensation(const ahl_robot::RobotPtr& robot);
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
    virtual const std::string& getTargetName() { return robot_->getName(); }

  private:
    ahl_robot::RobotPtr robot_;
    Eigen::MatrixXd b_;
    std::vector<std::string> mnp_name_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_FRICTION_COMPENSATION_HPP */
