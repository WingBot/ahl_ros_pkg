#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/mobility/mecanum_wheel.hpp"

using namespace ahl_ctrl;

MecanumWheel::MecanumWheel(double tread_width, double wheel_base, double wheel_radius)
  : tread_width_(tread_width),
    wheel_base_(wheel_base),
    wheel_radius_(wheel_radius)
{
  double l1 = 0.5 * tread_width_;
  double l2 = 0.5 * wheel_base_;

  decomposer_.resize(4, 3);
  decomposer_ <<
    1.0,  1.0, -(l1 + l2),
    1.0, -1.0,   l1 + l2,
    1.0, -1.0, -(l1 + l2),
    1.0, 1.0,    l1 + l2;

  if(wheel_radius_ == 0.0)
  {
    throw ahl_ctrl::Exception("MecanumWheel::MecanumWheel", "Wheel radius cannot be set to zero.");
  }

  decomposer_ = (1.0 / wheel_radius_) * decomposer_;
}

void MecanumWheel::computeWheelVelocity(const Eigen::Vector3d& v_base, Eigen::VectorXd& v_wheel)
{
  v_wheel = decomposer_ * v_base;
}
