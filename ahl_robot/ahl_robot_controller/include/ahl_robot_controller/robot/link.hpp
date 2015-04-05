#ifndef __AHL_ROBOT_CONTROLLER_LINK_HPP
#define __AHL_ROBOT_CONTROLLER_LINK_HPP

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/robot/joint.hpp"

namespace ahl_robot
{

  class Joint;
  typedef boost::shared_ptr<Joint> JointPtr;
  typedef std::map<std::string, JointPtr> Joints;
  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;
  typedef std::map<std::string, LinkPtr> Links;

  class Link
  {
  public:
    Link(const std::string& name, const Eigen::Matrix3d& I, double M,
         double x, double y, double z, double roll, double pitch, double yaw,
         double x_com, double y_com, double z_com,
         double roll_com, double pitch_com, double yaw_com);

    void setJoint(const JointPtr& joint);
    void addChildJoint(const JointPtr& joint);

    const std::string& getName() const
    {
      return name_;
    }

    const Eigen::Matrix3d& getI() const
    {
      return I_;
    }

    const Eigen::Matrix4d& getT() const
    {
      return T_;
    }

    const Eigen::Matrix4d& getCOM() const
    {
      return COM_;
    }

    double getM()
    {
      return M_;
    }

    const JointPtr getJoint() const
    {
      return joint_;
    }

    const Joints getChildJoints() const
    {
      return child_joints_;
    }

    void print();

  private:
    std::string name_;

    Eigen::Matrix3d I_;
    double M_;

    Eigen::Matrix4d T_org_;
    Eigen::Matrix4d T_;
    Eigen::Matrix4d COM_;

    JointPtr joint_;
    Joints child_joints_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_LINK_HPP */
