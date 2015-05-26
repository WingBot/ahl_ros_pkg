#include <ros/ros.h>
#include "ahl_robot/definition.hpp"
#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/manipulator.hpp"
#include "ahl_robot/utils/math.hpp"

using namespace ahl_robot;

Manipulator::Manipulator()
  : name(""), dof(0)
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

  J0 = Eigen::MatrixXd::Zero(6, dof);
  T_abs_.resize(dof + 1);
  for(unsigned int i = 0; i < T_abs_.size(); ++i)
  {
    T_abs_[i] = Eigen::Matrix4d::Identity();
  }
  Pin_.resize(dof + 1);
  for(unsigned int i = 0; i < Pin_.size(); ++i)
  {
    Pin_[i] = Eigen::Vector3d::Zero();
  }

  q = init_q;
  this->computeForwardKinematics();
  pre_q  = q;
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
  this->computeBasicJacobian(link[link.size() - 1]->name, J0);

  std::cout << J0 * dq << std::endl;
}

void Manipulator::computeBasicJacobian(const std::string& name, Eigen::MatrixXd& J)
{
  if(link.size() != T.size())
  {
    std::stringstream msg;
    msg << "link.size() != T.size()" << std::endl
        << "  link.size : " << link.size() << std::endl
        << "  T.size    : " << T.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  if(link.size() != dof + 1)
  {
    std::stringstream msg;
    msg << "mnp->link.size() != dof + 1" << std::endl
        << "The number of link should be DOF + 1,"
        << "including virtual link which is attached to operational point frame and following the last link." << std::endl
        << "  mnp : " << name << std::endl
        << "  link.size : " << link.size() << std::endl
        << "  dof : " << dof;
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  // Find the index of the link of the name
  int idx = -1;
  for(unsigned int i = 0; i < link.size(); ++i)
  {
    if(link[i]->name == name)
    {
      idx = i;
      break;
    }
  }

  if(idx < 0)
  {
    std::stringstream msg;
    msg << "Could not find link : " << name << ".";
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  this->computeBasicJacobian(idx, J);
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

    link[i]->tf->transform(q.coeff(idx), T[i]);
    ++idx;
  }

  // Absolute transformation matrix
  this->computeTabs();

  // Compute distance between i-th link and end-effector w.r.t base
  if(T_abs_.size() != Pin_.size())
  {
    std::stringstream msg;
    msg << "T_abs_.size() != Pin_.size()" << std::endl
        << "  T_abs_.size    : " << T_abs_.size() << std::endl
        << "  Pin_.size : " << Pin_.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
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
    math::calculateInverseTransformationMatrix(T_abs_[i], Tib);
    Eigen::MatrixXd Pin = Tib * Pbn;
    Pin_[i] = Pin.block(0, 0, 3, 1);
  }

  // Compute end-effector position and orientation
  xp = T_abs_[T_abs_.size() - 1].block(0, 3, 3, 1);
  Eigen::Matrix3d R = T_abs_[T_abs_.size() - 1].block(0, 0, 3, 3);
  xr = R;

  // Compute velocity of generalized coordinates
  this->computeVelocity();
}

void Manipulator::computeTabs()
{
  if(T_abs_.size() != T.size())
  {
    std::stringstream msg;
    msg << "T_abs_.size() != T.size()" << std::endl
        << "  T_abs_.size : " << T_abs_.size() << std::endl
        << "  T.size      : " << T.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
  }
  else if(T_abs_.size() == 0 || T.size() == 0)
  {
    std::stringstream msg;
    msg << "T_abs_.size() == 0 || T.size() == 0" << std::endl
        << "  T_abs_.size    : " << T_abs_.size() << std::endl
        << "  T.size : " << T.size();
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeTabs", msg.str());
  }

  T_abs_.front() = T.front();
  for(unsigned int i = 1; i < T_abs_.size(); ++i)
  {
    T_abs_[i] = T_abs_[i - 1] * T[i];
  }
}

void Manipulator::computeBasicJacobian(int idx, Eigen::MatrixXd& J)
{
  J = Eigen::MatrixXd::Zero(6, dof);

  // Compute basic jacobian
  for(unsigned int i = 0; i < idx; ++i)
  {
    if(link[i]->ep) // joint_type is prismatic
    {
      J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link[i]->tf->axis();
      J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * Eigen::Vector3d::Zero();
    }
    else // joint_type is revolute
    {
      J.block(0, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link[i]->tf->axis().cross(Pin_[i]);
      J.block(3, i, 3, 1) = T_abs_[i].block(0, 0, 3, 3) * link[i]->tf->axis();
    }
  }

  // Pne is the distance between end-effector and last joint w.r.t base
  Eigen::MatrixXd J_Pne = Eigen::MatrixXd::Identity(6, 6);
  Eigen::Vector3d Pne;
  if(T_abs_.size() - 2 >= 0.0)
  {
    Pne = xp - T_abs_[T_abs_.size() - 2].block(0, 3, 3, 1);
  }
  else
  {
    std::stringstream msg;
    msg << "T_abs_.size() <= 1" << std::endl
        << "Manipulator doesn't have enough links." << std::endl;
    throw ahl_robot::Exception("ahl_robot::Manipulator::computeBasicJacobian", msg.str());
  }

  // Compute basic jacobian associated with end-effector
  Eigen::Matrix3d Pne_cross;
  Pne_cross <<           0.0,  Pne.coeff(2), -Pne.coeff(1),
               -Pne.coeff(2),           0.0,  Pne.coeff(0),
                Pne.coeff(1), -Pne.coeff(0),           0.0;
  J_Pne.block(0, 3, 3, 3) = Pne_cross;
  J = J_Pne * J;
}

void Manipulator::computeVelocity()
{
  double dt = (time_ - pre_time_) * 0.001;
  if(dt > 0.0)
  {
    dq  = (q - pre_q) / dt;
  }
  else
  {
    dq = Eigen::VectorXd::Zero(dq.rows());
  }

  pre_q  = q;
  pre_time_ = time_;
}
