#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/utils/math.hpp"
#include "ahl_robot_controller/robot/link.hpp"

using namespace ahl_robot;

Link::Link(const std::string& name, const Eigen::Matrix3d& I, double M,
           double x, double y, double z, double roll, double pitch, double yaw,
           double x_com, double y_com, double z_com,
           double roll_com, double pitch_com, double yaw_com)
  : name_(name), I_(I), M_(M)
{
  Eigen::Matrix3d R;

  utils::convertRPYToRotationMatrix(roll, pitch, yaw, R);
  T_.coeffRef(0, 3) = x;
  T_.coeffRef(1, 3) = y;
  T_.coeffRef(2, 3) = z;
  T_.block(0, 0, 3, 3) = R;
  T_.block(3, 0, 1, 4) = utils::getLastRowOfTransformationMatrix();
  T_org_ = T_;

  utils::convertRPYToRotationMatrix(roll_com, pitch_com, yaw_com, R);
  COM_.coeffRef(0, 3) = x_com;
  COM_.coeffRef(1, 3) = y_com;
  COM_.coeffRef(2, 3) = z_com;
  COM_.block(0, 0, 3, 3) = R;
  COM_.block(3, 0, 1, 4) = utils::getLastRowOfTransformationMatrix();
}

void Link::setJoint(const JointPtr& joint)
{
  if(!joint)
  {
    std::stringstream msg;
    msg << name_ << " could not set null joint ptr.";

    throw ahl_robot::Exception("Link::setJoint", msg.str());
  }

  joint_ = joint;
}

void Link::addChildJoint(const JointPtr& joint)
{
  if(!joint)
  {
    std::stringstream msg;
    msg << name_ << " could not set null joint ptr.";

    throw ahl_robot::Exception("Link::addChildJoint", msg.str());
  }

  child_joints_[joint->getName()] = joint;
}

void Link::setChildJoints(const Joints& joints)
{
  child_joints_ = joints;
}

void Link::print()
{
  std::cout << "<< Link name   : " << name_ << " >>" << std::endl
            << "Inertia matrix : " << std::endl << I_ << std::endl
            << "Origin w.r.t world : " << std::endl << T_ << std::endl
            << "Center of Mass w.r.t world : " << std::endl << COM_ << std::endl
            << "Mass           : " << M_ << std::endl;

  if(joint_)
    std::cout << "parent joint   : " << joint_->getName() << std::endl;
  else
    std::cout << "parent joint   : N/A" << std::endl;

  Joints::iterator it;
  if(child_joints_.begin() == child_joints_.end())
    std::cout << "child joint    : N/A" << std::endl;

  for(it = child_joints_.begin(); it != child_joints_.end(); ++it)
  {
    std::cout << "child joint    : " << it->first << std::endl;
  }
}
