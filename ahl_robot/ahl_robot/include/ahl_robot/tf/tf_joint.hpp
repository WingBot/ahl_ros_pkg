#ifndef __AHL_ROBOT_TF_JOINT_HPP
#define __AHL_ROBOT_TF_JOINT_HPP

#include <boost/shared_ptr.hpp>
#include "ahl_robot/robot/link.hpp"
#include "ahl_robot/tf/tf_link.hpp"

namespace ahl_robot
{

  class Link;
  class TfLink;
  typedef boost::shared_ptr<TfLink> TfLinkPtr;
  typedef std::map<std::string, TfLinkPtr> TfLinkMap;

  class TfJoint
  {
  public:
    TfJoint()
      : name(""), type(Link::FIXED), q(0.0), pre_q(0.0), q_min(0.0), q_max(0.0),
        dq(0.0), dq_max(0.0), tau(0.0), tau_max(0.0)
    {
      T = Eigen::Matrix4d::Identity();
      axis = Eigen::Vector3d::Zero();
    }

    void copyTo(LinkPtr& link)
    {
      link->type = type;

      link->T_org = T;
      link->T = T;
      link->axis = axis;

      link->q = q;
      link->pre_q = pre_q;
      link->q_min = q_max;
      link->dq = dq_max;

      link->tau = tau;
      link->tau_max = tau_max;
    }

    std::string name;
    Link::JointType type;

    //Eigen::Matrix4d T_org; // Frame attached to joint in parent joint frame
    Eigen::Matrix4d T; // Initial frame attached to joint in parent joint frame
    Eigen::Vector3d axis; // Axis in joint frame

    double q;
    double pre_q;
    double q_min;
    double q_max;

    double dq;
    double dq_max;

    double tau;
    double tau_max;

    TfLinkPtr link;
    TfLinkPtr parent_link;
  };

}

#endif /* __AHL_ROBOT_TF_JOINT_HPP */
