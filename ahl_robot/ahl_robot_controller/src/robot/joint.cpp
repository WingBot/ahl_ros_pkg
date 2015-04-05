#include <cmath>
#include <climits>
#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/utils/math.hpp"
#include "ahl_robot_controller/robot/joint.hpp"

using namespace ahl_robot;

Joint::Joint(const std::string& name, bool is_revolute,
              double x, double y, double z, double roll, double pitch, double yaw,
             const Eigen::Vector3d& axis,
             double q_min, double q_max, double dq_max, double tau_max)
  : name_(name), is_revolute_(is_revolute), axis_(axis),
    q_(0.0), pre_q_(0.0), q_min_(q_min), q_max_(q_max),
    dq_(0.0), pre_dq_(0.0), dq_max_(dq_max),
    ddq_(0.0), tau_(0.0), tau_max_(tau_max),
    locked_(false)
{
  Eigen::Matrix3d R;

  utils::convertRPYToRotationMatrix(roll, pitch, yaw, R);
  T_.coeffRef(0, 3) = x;
  T_.coeffRef(1, 3) = y;
  T_.coeffRef(2, 3) = z;
  T_.block(0, 0, 3, 3) = R;
  T_.block(3, 0, 1, 4) = utils::getLastRowOfTransformationMatrix();
  T_org_ = T_;
}

void Joint::setLink(const LinkPtr& link)
{
  if(!link)
  {
    std::stringstream msg;
    msg << name_ << " could not set null link ptr.";

    throw ahl_robot::Exception("Link::setLink", msg.str());
  }

  link_ = link;
}

void Joint::setParentLink(const LinkPtr& link)
{
  if(!link)
  {
    std::stringstream msg;
    msg << name_ << " could not set null link ptr.";

    throw ahl_robot::Exception("Link::setLink", msg.str());
  }

  parent_link_ = link;
}

void Joint::print()
{
  std::cout << "<< Joint name : " << name_ << " >>" << std::endl;

  if(is_revolute_)
    std::cout << "type : revolute" << std::endl;
  else
    std::cout << "type : prismatic" << std::endl;

  std::cout << "Origin w.r.t world : " << std::endl << T_ << std::endl
            << "Axis : " << std::endl << axis_ << std::endl
            << "q : " << q_
            << ", q_min : " << q_min_
            << ", q_max : " << q_max_ << std::endl
            << "dq : " << dq_ 
            << ", dq_max : " << dq_max_ << std::endl
            << "ddq : " << ddq_ << std::endl
            << "tau : " << tau_ 
            << ", tau_max : " << tau_max_ << std::endl;

  if(locked_)
    std::cout << "locked : true" << std::endl;
  else
    std::cout << "locked : false" << std::endl;

  if(parent_link_)
    std::cout << "parent link : " << parent_link_->getName() << std::endl;
  else
    std::cout << "parent link : N/A" << std::endl;

  if(link_)
    std::cout << "child link  : " << link_->getName() << std::endl;
  else
    std::cout << "child link  : N/A" << std::endl;
}
