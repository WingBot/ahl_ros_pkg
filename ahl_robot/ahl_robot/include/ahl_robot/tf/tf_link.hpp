#ifndef __AHL_ROBOT_TF_LINK_HPP
#define __AHL_ROBOT_TF_LINK_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Link;
  class TfJoint;
  typedef boost::shared_ptr<TfJoint> TfJointPtr;
  typedef std::map<std::string, TfJointPtr> TfJointMap;

  class TfLink
  {
  public:
    TfLink()
      : name(""), m(0.0)
    {
      I = Eigen::Matrix3d::Zero();
      T = com = Eigen::Matrix4d::Identity();
    }

    void copyTo(LinkPtr& link)
    {
      link->name = name;
      link->m = m;
      link->I = I;
      link->com = com;
    }

    std::string name;

    double m;
    Eigen::Matrix3d I;

    //Eigen::Matrix4d T_org; // Initial frame attached to link in previous link frame or world
    Eigen::Matrix4d T; // Frame attached to link in previous link frame or world
    Eigen::Matrix4d com; // Frame attached to center of mass in link frame 
    //Eigen::Vector3d axis;

    TfJointPtr joint;
    TfJointMap child_joint;
  };

}

#endif /* __AHL_ROBOT_TF_LINK_HPP */
