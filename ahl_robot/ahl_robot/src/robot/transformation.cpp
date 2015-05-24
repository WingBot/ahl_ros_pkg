#include "ahl_robot/robot/transformation.hpp"

using namespace ahl_robot;

Transformation::Transformation()
{
  T_ = Eigen::Matrix4d::Identity();
}

RevoluteX::RevoluteX()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 1, 0, 0;
}

const Eigen::Matrix4d& RevoluteX::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

RevoluteY::RevoluteY()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 0, 1, 0;
}

const Eigen::Matrix4d& RevoluteY::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

RevoluteZ::RevoluteZ()
{
  R_ = Eigen::Matrix3d::Identity();
  axis_ << 0, 0, 1;
}

const Eigen::Matrix4d& RevoluteZ::T(double q)
{
  R_ = Eigen::AngleAxisd(q, axis_);
  T_.block(0, 0, 3, 3) = R_;
  return T_;
}

const Eigen::Matrix4d& PrismaticX::T(double q)
{
  T_.coeffRef(0, 3) = q;
  return T_;
}

const Eigen::Matrix4d& PrismaticY::T(double q)
{
  T_.coeffRef(1, 3) = q;
  return T_;
}

const Eigen::Matrix4d& PrismaticZ::T(double q)
{
  T_.coeffRef(2, 3) = q;
  return T_;
}
