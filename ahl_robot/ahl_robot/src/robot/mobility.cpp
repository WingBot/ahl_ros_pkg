#include <ahl_digital_filter/pseudo_differentiator.hpp>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/definition.hpp"
#include "ahl_robot/robot/mobility.hpp"

using namespace ahl_robot;

Mobility::Mobility()
  : update_rate(0.01),
    type(""),
    command(""),
    cutoff_frequency_base(20.0),
    cutoff_frequency_wheel(100.0),
    tread_width(0.0),
    wheel_base(0.0),
    wheel_radius(0.0),
    updated_pos_(false),
    updated_wheel_(false)
{
  p = Eigen::Vector3d::Zero();
  r = Eigen::Matrix3d::Identity();
  v = Eigen::Vector3d::Zero();
  w = Eigen::Vector3d::Zero();
  last_ori_update_time_ = ros::Time::now();
}

void Mobility::init()
{
  if(type == ahl_robot::mobility::type::MECANUM_WHEEL)
  {
    q  = Eigen::Vector4d::Zero();
    dq = Eigen::Vector4d::Zero();
  }
  else
  {
    std::stringstream msg;
    msg << "Mobility type : " << type << " is not supported.";
    throw ahl_robot::Exception("Mobility::init", msg.str());
  }

  differentiator_pos_ = ahl_filter::DifferentiatorPtr(
    new ahl_filter::PseudoDifferentiator(update_rate, cutoff_frequency_base));
  differentiator_wheel_ = ahl_filter::DifferentiatorPtr(
    new ahl_filter::PseudoDifferentiator(update_rate, cutoff_frequency_wheel));
}

void Mobility::updateBase(const Eigen::Vector3d& p_msr, const Eigen::Quaternion<double>& r_msr)
{
  if(!updated_pos_)
  {
    p = p_msr;
    r = r_msr;
    differentiator_pos_->init(p, v);
    updated_pos_ = true;
    return;
  }

  p = p_msr;

  differentiator_pos_->apply(p_msr);
  differentiator_pos_->copyDerivativeValueTo(this->v);

  Eigen::Matrix3d R = r.toRotationMatrix();
  Eigen::Matrix3d R_msr = r_msr.toRotationMatrix();
  Eigen::Quaternion<double> quat;
  quat = R * R_msr.transpose();

  r = r_msr;

  double norm = sqrt(quat.x() * quat.x() + quat.y() * quat.y() + quat.z() * quat.z());

  if(norm == 0.0)
  {
    if((last_ori_update_time_ - ros::Time::now()).toSec() > update_rate * 10.0)
    {
      w = Eigen::Vector3d::Zero();
      return;
    }
    else
    {
      return;
    }
  }

  double rad = 2 * acos(quat.w());
  double coeff = rad / norm / update_rate;
  w << coeff * quat.x(), coeff * quat.y(), coeff * quat.z();

  last_ori_update_time_ = ros::Time::now();
}

void Mobility::updateWheel(const Eigen::VectorXd& q_msr)
{
  if(q.rows() != q_msr.rows())
  {
    std::stringstream msg;
    msg << "q.rows() != q_msr.rows()" << std::endl
        << "  q.rows     : " << q.rows() << std::endl
        << "  q_msr.rows : " << q_msr.rows();
    throw ahl_robot::Exception("Mobility::update", msg.str());
  }

  if(!updated_wheel_)
  {
    q = q_msr;
    differentiator_wheel_->init(q, dq);
    updated_wheel_ = true;
    return;
  }

  q = q_msr;

  differentiator_wheel_->apply(this->q);
  differentiator_wheel_->copyDerivativeValueTo(this->dq);
}
