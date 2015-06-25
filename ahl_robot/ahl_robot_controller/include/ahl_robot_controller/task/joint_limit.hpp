#ifndef __AHL_ROBOT_CONTROLLER_JOINT_LIMIT_HPP
#define __AHL_ROBOT_CONTROLLER_JOINT_LIMIT_HPP

#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class JointLimit : public Task
  {
  public:
    JointLimit(const ahl_robot::ManipulatorPtr& mnp, double threshold);
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
    virtual bool haveNullSpace() { return true; }

  private:
    Eigen::VectorXd q_max_;
    Eigen::VectorXd q_min_;

    double kp_;
    double kv_;
    double threshold_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_JOINT_LIMIT_HPP */
