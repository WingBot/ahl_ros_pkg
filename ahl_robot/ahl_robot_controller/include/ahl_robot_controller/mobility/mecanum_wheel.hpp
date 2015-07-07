#ifndef __AHL_ROBOT_CONTROLLER_MECANUM_WHEEL_HPP
#define __AHL_ROBOT_CONTROLLER_MECANUM_WHEEL_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/param.hpp"

namespace ahl_ctrl
{

  // MecanumWheel class assumes that all initial orientations of
  // frames attached to each wheel are the same
  // and that the rollers angle w.r.t wheel axis is 45 deg.

  class MecanumWheel
  {
  public:
    MecanumWheel(double tread_width, double wheel_base, double wheel_radius);

    // v.coeff(0) : vx, v.coeff(1) : vy, v.coeff(2) : vyaw
    void computeWheelVelocity(const Eigen::Vector3d& v_base, Eigen::VectorXd& v_wheel);

  private:
    double tread_width_; // [m]
    double wheel_base_; // [m]
    double wheel_radius_; // [m]

    Eigen::MatrixXd decomposer_;
  };

  typedef boost::shared_ptr<MecanumWheel> MecanumWheelPtr;
}

#endif /* __AHL_ROBOT_MECANUM_WHEEL_HPP */
