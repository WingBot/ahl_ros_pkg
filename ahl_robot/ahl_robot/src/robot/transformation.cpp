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

void RevoluteX::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 0, 3, 3) = this->T(q).block(0, 0, 3, 3);
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

void RevoluteY::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 0, 3, 3) = this->T(q).block(0, 0, 3, 3);
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

void RevoluteZ::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 0, 3, 3) = this->T(q).block(0, 0, 3, 3);
}

PrismaticX::PrismaticX()
{
  axis_ << 1, 0, 0;
}

const Eigen::Matrix4d& PrismaticX::T(double q)
{
  T_.coeffRef(0, 3) = q;
  return T_;
}

void PrismaticX::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 3, 3, 1) = this->T(q).block(0, 3, 3, 1);
}

PrismaticY::PrismaticY()
{
  axis_ << 0, 1, 0;
}

const Eigen::Matrix4d& PrismaticY::T(double q)
{
  T_.coeffRef(1, 3) = q;
  return T_;
}

void PrismaticY::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 3, 3, 1) = this->T(q).block(0, 3, 3, 1);
}

PrismaticZ::PrismaticZ()
{
  axis_ << 0, 0, 1;
}

const Eigen::Matrix4d& PrismaticZ::T(double q)
{
  T_.coeffRef(2, 3) = q;
  return T_;
}

void PrismaticZ::transform(double q, Eigen::Matrix4d& T)
{
  T.block(0, 3, 3, 1) = this->T(q).block(0, 3, 3, 1);
}
