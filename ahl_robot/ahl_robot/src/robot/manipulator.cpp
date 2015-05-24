#include "ahl_robot/definition.hpp"
#include "ahl_robot/robot/manipulator.hpp"

using namespace ahl_robot;

Manipulator::Manipulator()
  : name(""), dof(0)
{
  xp  = Eigen::Vector3d::Zero();
  dxp = Eigen::Vector3d::Zero();
  xr.w() = 1.0;
  xr.x() = 0.0;
  xr.y() = 0.0;
  xr.z() = 0.0;
  dxr  = Eigen::Vector3d::Zero();
  Rn = Eigen::Matrix3d::Zero();
}

void Manipulator::print()
{
  std::cout << "name : " << name << std::endl
            << "dof : " << dof << std::endl
            << "q : " << std::endl << q << std::endl
            << "dq : " << std::endl << dq << std::endl
            << "xp : " << std::endl << xp << std::endl
            << "dxp : " << std::endl << dxp << std::endl
            << "xr : " << std::endl
            << xr.w() << std::endl
            << xr.x() << std::endl
            << xr.y() << std::endl
            << xr.z() << std::endl
            << "dxr : " << std::endl << dxr << std::endl;

  for(unsigned int i = 0; i < T.size(); ++i)
  {
    std::cout << "T[" << i << "] :" << std::endl << T[i] << std::endl;
  }

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    link[i]->print();
  }
}

void Manipulator::computeFK()
{
  int idx = 0;

  for(unsigned int i = 0; i < link.size(); ++i)
  {
    if(link[i]->joint_type == joint::FIXED)
      continue;

    T[i] = link[i]->tf->T(q.coeff(idx)) * link[i]->T_org;
    ++idx;
  }

  Eigen::Matrix4d Tn = Eigen::Matrix4d::Identity();
  for(unsigned int i = 0; i < link.size(); ++i)
  {
    Tn *= T[link.size() - 1 - i];
  }

  xp = Tn.block(0, 3, 3, 1);
  Eigen::Matrix3d R = Tn.block(0, 0, 3, 3);
  xr = R;
}
