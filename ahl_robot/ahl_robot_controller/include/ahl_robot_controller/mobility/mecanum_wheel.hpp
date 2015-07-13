#ifndef __AHL_ROBOT_CONTROLLER_MECANUM_WHEEL_HPP
#define __AHL_ROBOT_CONTROLLER_MECANUM_WHEEL_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/param.hpp"
#include "ahl_robot_controller/mobility/mobility_controller.hpp"

namespace ahl_ctrl
{

  // MecanumWheel class assumes that all initial orientations of
  // frames attached to each wheel are the same
  // and that the rollers angle w.r.t wheel axis is 45 deg.

  class MecanumWheel : public MobilityController
  {
  public:
    MecanumWheel(const ahl_robot::MobilityPtr& mobility, const ParamPtr& param);

    void computeBaseVelocityFromTorque(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& tau, Eigen::VectorXd& v_base);
    void computeWheelVelocityFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& v_wheel);
    void computeWheelTorqueFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& tau_wheel);

  private:
    ahl_robot::MobilityPtr mobility_;
    ParamPtr param_;
    Eigen::MatrixXd decomposer_;
  };

  typedef boost::shared_ptr<MecanumWheel> MecanumWheelPtr;
}

#endif /* __AHL_ROBOT_MECANUM_WHEEL_HPP */
