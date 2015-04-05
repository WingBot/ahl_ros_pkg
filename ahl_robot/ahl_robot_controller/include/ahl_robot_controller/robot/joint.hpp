#ifndef __AHL_ROBOT_CONTROLLER_JOINT_HPP
#define __AHL_ROBOT_CONTROLLER_JOINT_HPP

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/robot/link.hpp"

namespace ahl_robot
{

  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;

  class Joint
  {
  public:
    Joint(const std::string& name, bool is_revolute,
          double x, double y, double z, double roll, double pitch, double yaw,
          const Eigen::Vector3d& axis,
          double q_min, double q_max, double dq_max, double tau_max);

    void setLink(const LinkPtr& link);
    void setParentLink(const LinkPtr& link);

    const std::string& getName() const
    {
      return name_;
    }

    bool isRevolute()
    {
      return is_revolute_;
    }

    const Eigen::Matrix4d& getT() const
    {
      return T_;
    }

    const LinkPtr getLink() const
    {
      return link_;
    }

    const LinkPtr getParentLink() const
    {
      return parent_link_;
    }

    void print();

  private:
    std::string name_;
    bool is_revolute_; // If false, it's prismatic

    Eigen::Matrix4d T_org_;
    Eigen::Matrix4d T_;
    Eigen::Vector3d axis_; // axis w.r.t this joint frame

    double q_; // generalized coordinate
    double pre_q_;
    double q_min_;
    double q_max_;

    double dq_; // velocity
    double pre_dq_;
    double dq_max_;

    double ddq_; // acceleration

    double tau_; // generalized force
    double tau_max_;

    bool locked_;

    LinkPtr link_;
    LinkPtr parent_link_;
  };

  typedef boost::shared_ptr<Joint> JointPtr;
  typedef std::map<std::string, JointPtr> Joints;
}

#endif /* __AHL_ROBOT_CONTROLLER_JOINT_HPP */
