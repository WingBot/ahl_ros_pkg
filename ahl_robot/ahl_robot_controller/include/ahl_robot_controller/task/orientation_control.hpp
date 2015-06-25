#ifndef __AHL_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP
#define __AHL_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP

#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/task/task.hpp"

namespace ahl_ctrl
{

  class OrientationControl : public Task
  {
  public:
    OrientationControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, double eigen_thresh);
    virtual void setGoal(const Eigen::MatrixXd& Rd);
    virtual void updateModel();
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
    virtual bool haveNullSpace() { return true; }

  private:
    bool updated_;

    std::string target_link_;
    Eigen::Matrix3d Rd_;

    int idx_;
    Eigen::MatrixXd Jw_;
    Eigen::Matrix3d lambda_inv_;
    Eigen::Matrix3d lambda_;
    Eigen::MatrixXd J_dyn_inv_;
    Eigen::MatrixXd I_;

    double eigen_thresh_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_ORIENTATION_CONTROL_HPP */
