#ifndef __AHL_ROBOT_CONTROLLER_JOINT_HPP
#define __AHL_ROBOT_CONTROLLER_JOINT_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/robot/link.hpp"

namespace ahl_robot
{

  class Joint
  {
  public:
    Joint();
    void print();

    std::string name;

    bool is_revolute; // if false, it's prismatic
    Eigen::Matrix4d org;
    Eigen::Matrix3d I;
    double M;

    Eigen::Vector3d axis;

    double gear_ratio;

    double q; // generalized coordinate
    double q_min;
    double q_max;

    double dq;
    double dq_max;

    double ddq;

    double tau; // generalized force
    double tau_max;

    bool locked;

    LinkPtr link;
    LinkPtr parent_link;
  };

  typedef boost::shared_ptr<Joint> JointPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_JOINT_HPP */
