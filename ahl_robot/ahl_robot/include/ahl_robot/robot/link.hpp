#ifndef __AHL_ROBOT_LINK_HPP
#define __AHL_ROBOT_LINK_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;
  typedef std::map<std::string, LinkPtr> Links;

  class TfLink;
  class TfJoint;

  class Link
  {
  public:
    enum JointType
    {
      FIXED, // 0 DOF
      OMNIDIRECTIONAL_BASE, // it can move along x and y axis, and rotate about z axis
      FREE, // 6 DOF like a base of flying robot
      REVOLUTE,
      PRISMATIC,
      JOINT_TYPE_NUM,
    };

    Link()
      : name(""), type(FIXED), m(0.0), q(0.0), pre_q(0.0), q_min(0.0), q_max(0.0),
        dq(0.0), dq_max(0.0), tau(0.0), tau_max(0.0)
    {
      I = Eigen::Matrix3d::Zero();
      T_org = T = com = Eigen::Matrix4d::Identity();
      axis = Eigen::Vector3d::Zero();
    }

    std::string name;
    JointType type;

    double m;
    Eigen::Matrix3d I;

    Eigen::Matrix4d T_org; // Initial frame attached to joint in parent joint frame
    Eigen::Matrix4d T; // Current frame attached to joint in parent joint frame
    Eigen::Matrix4d com; // Frame center of mass in joint frame
    Eigen::Vector3d axis; // Direction of joint

    double q;
    double pre_q;
    double q_min;
    double q_max;

    double dq;
    double dq_max;

    double tau;
    double tau_max;
  };

}

#endif /* __AHL_ROBOT_LINK_HPP */
