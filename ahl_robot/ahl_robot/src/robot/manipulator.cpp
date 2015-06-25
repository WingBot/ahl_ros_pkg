#include <ros/ros.h>
#include <ahl_digital_filter/pseudo_differentiator.hpp>
#include "ahl_robot/definition.hpp"
#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/manipulator.hpp"
#include "ahl_robot/utils/math.hpp"

using namespace ahl_robot;

Manipulator::Manipulator()
  : name(""), dof(0), updated_joint_(false)
{
  xp  = Eigen::Vector3d::Zero();
  xr.w() = 1.0;
  xr.x() = 0.0;
  xr.y() = 0.0;
  xr.z() = 0.0;
  time_  = ros::Time::now().toNSec() * 0.001 * 0.001;
  pre_time_ = time_;
}

void Manipulator::init(unsigned int init_dof, const Eigen::VectorXd& init_q)
{
  if(init_dof != init_q.rows())
  {
    std::stringstream msg;
    msg << "dof != init_q.rows()" << std::endl
        << "  dof           : " << init_dof << std::endl
        << "  init_q.rows() : " << init_q.rows();
    throw ahl_robot::Exception("ahl_robot::Manipulator::init", msg.str());
  }

  dof = init_dof;

  // Resize vectors and matrices
  q     = Eigen::VectorXd::Zero(dof);
  pre_q = Eigen::VectorXd::Zero(dof);
  dq    = Eigen::VectorXd::Zero(dof);

  T_abs.resize(dof + 1);
  for(unsigned int i = 0; i < T_abs.size(); ++i)
  {
    T_abs[i] = Eigen::Matrix4d::Identity();
  }
  C_abs_.resize(dof + 1);
  for(unsigned int i = 0; i < C_abs_.size(); ++i)
  {
    C_abs_[i] = Eigen::Matrix4d::Identity();
  }
  Pin_.resize(dof + 1);
  for(unsigned int i = 0; i < Pin_.size(); ++i)
  {
    Pin_[i] = Eigen::Vector3d::Zero();
  }

  q = init_q;

  differentiator_ = ahl_filter::DifferentiatorPtr(
    new ahl_filter::PseudoDifferentiator(this->q, this->dq, 0.001, 30.0));
  this->computeForwardKinematics();

  pre_q  = q;

  J0.resize(link.size());
  M.resize(dof, dof);
  M_inv.resize(dof, dof);

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    name_to_idx[link[i]->name] = i;
  }
}

void Manipulator::update(const Eigen::VectorXd& q_msr)
{
  time_ = ros::Time::now().toNSec() * 0.001 * 0.001;

  if(q_msr.rows() != dof)
  {
    std::stringstream msg;
    msg << "q.rows() != dof" << std::endl
        << "  q.rows   : " << q_msr.rows() << std::endl
        << "  dof : " << dof;
    throw ahl_robot::Exception("ahl_robot::Manipulator::update", msg.str());
  }

  q = q_msr;
  this->computeForwardKinematics();
  //this->computeBasicJacobian();
  //this->computeMassMatrix();

  updated_joint_ = true;
}

void Manipulator::computeBasicJacobian()
{
  if(J0.size() != link.size())
  {
    std::stringstream msg;
    msg << "J0.size() != link.size()" << std::endl
        << "  J0.size   : " << J0.size() << std::endl
        << "  link.size : " << link.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    this->computeBasicJacobian(i, J0[i]);
  }
}

void Manipulator::computeMassMatrix()
{
  M = Eigen::MatrixXd::Zero(M.rows(), M.cols());

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    Eigen::MatrixXd Jv = J0[i].block(0, 0, 3, J0[i].cols());
    Eigen::MatrixXd Jw = J0[i].block(3, 0, 3, J0[i].cols());

    M += link[i]->m * Jv.transpose() * Jv + Jw.transpose() * link[i]->I * Jw;
  }

  M_inv = M.inverse();
}

bool Manipulator::reached(const Eigen::VectorXd& qd, double threshold)
{
  if(qd.rows() != q.rows())
  {
    std::stringstream msg;
    msg << "qd.rows() != q.rows()" << std::endl
        << "  qd.rows() : " << qd.rows() << std::endl
        << "  q.rows()  : " << q.rows();
    throw ahl_robot::Exception("Manipulator::reached", msg.str());
  }

  if((q - qd).norm() < threshold)
  {
    return true;
  }

  return false;
}

void Manipulator::print()
{
  std::cout << "name : " << name << std::endl
            << "dof : " << dof << std::endl
            << "q : " << std::endl << q << std::endl
            << "pre_q : " << std::endl << pre_q << std::endl
            << "dq : " << std::endl << dq << std::endl
            << "xp : " << std::endl << xp << std::endl
            << "xr : " << std::endl
            << xr.w() << std::endl
            << xr.x() << std::endl
            << xr.y() << std::endl
            << xr.z() << std::endl;

  for(unsigned int i = 0; i < T.size(); ++i)
  {
    std::cout << "T[" << i << "] :" << std::endl << T[i] << std::endl;
  }

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    link[i]->print();
  }
}

void Manipulator::computeForwardKinematics()
{
  int idx = 0;

  // Relative transformation matrix
  for(unsigned int i = 0; i < link.size(); ++i)
  {
    if(link[i]->joint_type == joint::FIXED)
      continue;

    link[i]->tf->transform(q.coeff(idx), link[i]->T_org, T[i]);
    ++idx;
  }

  // Absolute transformation matrix
  this->computeTabs();
  this->computeCabs();

  // Compute distance between i-th link and end-effector w.r.t base
  if(T_abs.size() != C_abs_.size())
  {
    std::stringstream msg;
    msg << "T_abs.size() != C_abs_.size()" << std::endl
        << "  T_abs.size    : " << T_abs.size() << std::endl
        << "  C_abs_.size : " << C_abs_.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeForwardKinematics", msg.str());
  }
  else if(T_abs.size() != Pin_.size())
  {
    std::stringstream msg;
    msg << "T_abs.size() != Pin_.size()" << std::endl
        << "  T_abs.size    : " << T_abs.size() << std::endl
        << "  Pin_.size : " << Pin_.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeForwardKinematics", msg.str());
  }
  else if(Pin_.size() == 0)
  {
    std::stringstream msg;
    msg << "Pin_.size() == 0";
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
  }

  Eigen::MatrixXd Pbn = Eigen::MatrixXd::Constant(4, 1, 1.0);
  Pbn.block(0, 0, 3, 1) = xp;

  for(unsigned int i = 0; i < Pin_.size(); ++i)
  {
    Eigen::Matrix4d Tib;
    math::calculateInverseTransformationMatrix(T_abs[i], Tib);
    Eigen::MatrixXd Pin = Tib * Pbn;
    Pin_[i] = Pin.block(0, 0, 3, 1);
  }

  // Compute end-effector position and orientation
  xp = T_abs[T_abs.size() - 1].block(0, 3, 3, 1);
  Eigen::Matrix3d R = T_abs[T_abs.size() - 1].block(0, 0, 3, 3);
  xr = R;

  // Compute velocity of generalized coordinates
  this->computeVelocity();
}

void Manipulator::computeTabs()
{
  if(T_abs.size() != T.size())
  {
    std::stringstream msg;
    msg << "T_abs.size() != T.size()" << std::endl
        << "  T_abs.size : " << T_abs.size() << std::endl
        << "  T.size      : " << T.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
  }
  else if(T_abs.size() == 0 || T.size() == 0)
  {
    std::stringstream msg;
    msg << "T_abs.size() == 0 || T.size() == 0" << std::endl
        << "  T_abs.size    : " << T_abs.size() << std::endl
        << "  T.size : " << T.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
  }

  T_abs.front() = T.front();
  for(unsigned int i = 1; i < T_abs.size(); ++i)
  {
    T_abs[i] = T_abs[i - 1] * T[i];
  }
}

void Manipulator::computeCabs()
{
  if(C_abs_.size() != T_abs.size())
  {
    std::stringstream msg;
    msg << "C_abs_.size() != T_abs.size()" << std::endl
        << "  C_abs_.size : " << C_abs_.size() << std::endl
        << "  T_abs.size : " << T_abs.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeCabs", msg.str());
  }
  else if(C_abs_.size() != link.size())
  {
    std::stringstream msg;
    msg << "C_abs_.size() != link.size()" << std::endl
        << "  C_abs_.size : " << C_abs_.size() << std::endl
        << "  link.size   : " << link.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeCabs", msg.str());
  }
  else if(C_abs_.size() == 0)
  {
    std::stringstream msg;
    msg << "C_abs_.size() == 0" << std::endl
        << "  C_abs_.size    : " << C_abs_.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeCabs", msg.str());
  }

  for(unsigned int i = 0; i < C_abs_.size(); ++i)
  {
    Eigen::Matrix4d Tlc = Eigen::Matrix4d::Identity();
    Tlc.block(0, 3, 3, 1) = link[i]->C;
    C_abs_[i] = T_abs[i] * Tlc;
  }
}

/*
void Manipulator::computeBasicJacobian()
{
  if(J0.size() != link.size())
  {
    std::stringstream msg;
    msg << "J0.size() != link.size()" << std::endl
        << "  J0.size   : " << J0.size() << std::endl
        << "  link.size : " << link.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    this->computeBasicJacobian(i, J0[i]);
  }
}
*/

void Manipulator::computeBasicJacobian(int idx, Eigen::MatrixXd& J)
{
  J = Eigen::MatrixXd::Zero(6, dof);

  if(idx < dof) // Not required to consider end-effector
  {
    for(unsigned int i = 0; i <= idx; ++i)
    {
      if(link[i]->ep) // joint_type is prismatic
      {
        J.block(0, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis();
        J.block(3, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * Eigen::Vector3d::Zero();
      }
      else // joint_type is revolute
      {
        Eigen::Matrix4d Tib;
        math::calculateInverseTransformationMatrix(T_abs[i], Tib);
        Eigen::Matrix4d Cin = Tib * C_abs_[idx];
        Eigen::Vector3d P = Cin.block(0, 3, 3, 1);

        J.block(0, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis().cross(P);
        J.block(3, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis();
      }
    }
  }
  else // Required to consider the offset of end-effector
  {
    --idx;
    for(unsigned int i = 0; i <= idx; ++i)
    {
      if(link[i]->ep) // joint_type is prismatic
      {
        J.block(0, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis();
        J.block(3, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * Eigen::Vector3d::Zero();
      }
      else // joint_type is revolute
      {
        Eigen::Matrix4d Tib;
        math::calculateInverseTransformationMatrix(T_abs[i], Tib);
        Eigen::Matrix4d Cin = Tib * C_abs_[idx];
        Eigen::Vector3d P = Cin.block(0, 3, 3, 1);

        J.block(0, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis().cross(P);
        J.block(3, i, 3, 1) = T_abs[i].block(0, 0, 3, 3) * link[i]->tf->axis();
      }
    }

    Eigen::MatrixXd J_Pne = Eigen::MatrixXd::Identity(6, 6);
    Eigen::Vector3d Pne;
    if(C_abs_.size() - 1 - 1 >= 0.0)
    {
      Pne = xp - C_abs_[C_abs_.size() - 1 - 1].block(0, 3, 3, 1);
    }
    else
    {
      std::stringstream msg;
      msg << "C_abs_.size() <= 1" << std::endl
          << "Manipulator doesn't have enough links." << std::endl;
      throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
    }

    Eigen::Matrix3d Pne_cross;
    Pne_cross <<           0.0,  Pne.coeff(2), -Pne.coeff(1),
                 -Pne.coeff(2),           0.0,  Pne.coeff(0),
                  Pne.coeff(1), -Pne.coeff(0),           0.0;
    J_Pne.block(0, 3, 3, 3) = Pne_cross;
    J = J_Pne * J;
  }
}

/*
void Manipulator::computeMassMatrix()
{
  M = Eigen::MatrixXd::Zero(M.rows(), M.cols());

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    Eigen::MatrixXd Jv = J0[i].block(0, 0, 3, J0[i].cols());
    Eigen::MatrixXd Jw = J0[i].block(3, 0, 3, J0[i].cols());

    M += link[i]->m * Jv.transpose() * Jv + Jw.transpose() * link[i]->I * Jw;
  }

  M_inv = M.inverse();
}
*/

void Manipulator::computeVelocity()
{
  if(updated_joint_)
    differentiator_->apply(this->q);
/*
  Eigen::VectorXd dq2;

  double dt = (time_ - pre_time_) * 0.001;
  dt = 0.001;

  if(dt > 0.0)
  {
    dq2  = (q - pre_q) / dt;
  }
  else
  {
    dq2 = Eigen::VectorXd::Zero(dq.rows());
  }

  pre_q  = q;
  pre_time_ = time_;

  std::cout << "dq : " << dq << std::endl << std::endl;
  std::cout << "dq2 : " << dq2 << std::endl << std::endl;
*/
}
