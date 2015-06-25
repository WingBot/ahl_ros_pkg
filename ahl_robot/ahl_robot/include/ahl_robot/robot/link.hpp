#ifndef __AHL_ROBOT_LINK_HPP
#define __AHL_ROBOT_LINK_HPP

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot/robot/transformation.hpp"

namespace ahl_robot
{
  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;

  class Link
  {
  public:
    Link ()
      : name(""), joint_type(""), parent(""), child(""), ep(false),
        m(0.0), q_min(0.0), q_max(0.0), dq_max(0.0), tau(0.0), tau_max(0.0)
    {
      T_org = Eigen::Matrix4d::Identity();
      C = Eigen::Vector3d::Zero();
      I = Eigen::Matrix3d::Zero();
    }

    void print()
    {
      std::cout << "name : " << name << std::endl
                << "joint_type : " << joint_type << std::endl
                << "parent : " << parent << std::endl
                << "child : " << child << std::endl
                << "ep    : " << ep << std::endl
                << "T_org : " << std::endl << T_org << std::endl
                << "C : " << std::endl << C << std::endl
                << "m : " << m << std::endl
                << "I : " << std::endl << I << std::endl
                << "q_min : " << q_min << std::endl
                << "q_max : " << q_max << std::endl
                << "dq_max : " << dq_max << std::endl
                << "tau : " << tau << std::endl
                << "tau_max : " << tau_max << std::endl;
    }

    std::string name;
    std::string joint_type;
    std::string parent;
    std::string child;
    bool ep;

    TransformationPtr tf;

    Eigen::Matrix4d T_org;
    Eigen::Vector3d C;

    double m;
    Eigen::Matrix3d I;

    double q_min;
    double q_max;
    double dq_max;
    double tau;
    double tau_max;
  };

}

#endif /* __AHL_ROBOT_LINK_HPP */
