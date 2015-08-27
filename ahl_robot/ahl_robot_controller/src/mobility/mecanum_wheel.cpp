#include "ahl_robot_controller/exception.hpp"
#include "ahl_robot_controller/mobility/mecanum_wheel.hpp"

using namespace ahl_ctrl;

MecanumWheel::MecanumWheel(const ahl_robot::MobilityPtr& mobility, const ParamBasePtr& param)
  : mobility_(mobility), param_(param)
{
  double l1 = 0.5 * mobility_->tread_width;
  double l2 = 0.5 * mobility_->wheel_base;

  decomposer_.resize(4, 3);
  decomposer_ <<
    1.0, -1.0, -(l1 + l2),
    1.0,  1.0,   l1 + l2,
    1.0,  1.0, -(l1 + l2),
    1.0, -1.0,   l1 + l2;

  if(mobility_->wheel_radius == 0.0)
  {
    throw ahl_ctrl::Exception("MecanumWheel::MecanumWheel", "Wheel radius cannot be set to zero.");
  }

  decomposer_ = (1.0 / mobility_->wheel_radius) * decomposer_;
}

void MecanumWheel::computeBaseVelocityFromTorque(
  const Eigen::MatrixXd& M, const Eigen::VectorXd& tau, Eigen::VectorXd& v_base)
{
  Eigen::MatrixXd D = 2.0 * M_PI * mobility_->cutoff_frequency_base * M;
  Eigen::MatrixXd M_inv = Eigen::MatrixXd::Zero(M.rows(), M.cols());

  for(unsigned int i = 0; i < M.rows(); ++i)
  {
    M_inv.coeffRef(i, i) = 1.0 / M.coeff(i, i);
  }

  Eigen::VectorXd v;
  if(M.rows() == 3 && M.cols() == 3)
  {
    v.resize(3);
    v.coeffRef(0) = mobility_->v.coeff(0);
    v.coeffRef(1) = mobility_->v.coeff(1);
    v.coeffRef(2) = mobility_->w.coeff(2);
  }
  else if(M.rows() == 6 && M.cols() == 6)
  {
    v.resize(6);
    v.block(0, 0, 3, 1) = mobility_->v;
    v.block(3, 0, 3, 1) = mobility_->w;
  }
  else
  {
    std::stringstream msg;
    msg << "Mass matrix size is invalid." << std::endl
        << "  M.rows : " << M.rows() << std::endl
        << "  M.cols : " << M.cols();
    throw ahl_ctrl::Exception("MecanumWheel::computeBaseVelocityFromTorque", msg.str());
  }

  Eigen::VectorXd a = M_inv * (tau - D * v);
  v_base = mobility_->v + mobility_->update_rate * a;
}

void MecanumWheel::computeWheelVelocityFromBaseVelocity(
  const Eigen::VectorXd& v_base, Eigen::VectorXd& v_wheel)
{
  if(v_base.rows() != 3)
  {
    std::stringstream msg;
    msg << "The size of vector representing base velocity is invalid." << std::endl
        << "  v_base.rows : " << v_base.rows();
    throw ahl_ctrl::Exception("MecanumWheel::computeWheelVelocityFromBaseVelocity", msg.str());
  }

  v_wheel = decomposer_ * v_base;
}

void MecanumWheel::computeWheelTorqueFromBaseVelocity(
  const Eigen::VectorXd& v_base, Eigen::VectorXd& tau_wheel)
{
  if(v_base.rows() != 3)
  {
    std::stringstream msg;
    msg << "The size of vector representing base velocity is invalid." << std::endl
        << "  v_base.rows : " << v_base.rows();
    throw ahl_ctrl::Exception("MecanumWheel::computeWheelTorqueFromBaseVelocity", msg.str());
  }

  Eigen::VectorXd v_wheel = decomposer_ * v_base;
  Eigen::VectorXd qd = mobility_->q + mobility_->update_rate * v_wheel;

  tau_wheel = - param_->getKpWheel() * (mobility_->q - qd) - param_->getKvWheel() * mobility_->dq;

  tau_wheel = 0.00175 * tau_wheel;
}
