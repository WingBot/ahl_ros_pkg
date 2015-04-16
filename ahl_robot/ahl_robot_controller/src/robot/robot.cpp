#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/utils/math.hpp"
#include "ahl_robot_controller/robot/robot.hpp"

using namespace ahl_robot;

Robot::Robot()
  : is_static_(false), initialized_(false)
{
  manipulator_ = ManipulatorPtr(new Manipulator());
  mobile_base_ = MobileBasePtr(new MobileBase());
}

void Robot::fix()
{
  is_static_ = true;
}

void Robot::unfix()
{
  is_static_ = false;
}

void Robot::addLink(const LinkPtr& link)
{
  if(!link)
  {
    std::stringstream msg;
    msg << "Could not add link \"" << link->getName() << "\".";

    throw ahl_robot::Exception("ahl_robot::Robot::addLink", msg.str());
  }

  links_[link->getName()] = link;
}

void Robot::addJoint(const JointPtr& joint)
{
  if(!joint)
  {
    std::stringstream msg;
    msg << "Could not add link \"" << joint->getName() << "\".";

    throw ahl_robot::Exception("ahl_robot::Robot::addJoint", msg.str());
  }

  joints_[joint->getName()] = joint;
}

void Robot::connectJointWithLink(const std::string& joint, const std::string& link)
{
  if(!joints_[joint])
  {
    std::stringstream msg;
    msg << joint << " is null.";

    throw ahl_robot::Exception("Robot::connectJointWithLink", msg.str());
  }
  else if(!links_[link])
  {
    std::stringstream msg;
    msg << link << " is null.";

    throw ahl_robot::Exception("Robot::connectJointWithLink", msg.str());
  }

  joints_[joint]->setLink(links_[link]);
  links_[link]->setJoint(joints_[joint]);
}

void Robot::connectJointWithParentLink(const std::string& joint, const std::string& link)
{
  if(!joints_[joint])
  {
    std::stringstream msg;
    msg << joint << " is null.";

    throw ahl_robot::Exception("Robot::connectJointWithLink", msg.str());
  }
  else if(!links_[link])
  {
    std::stringstream msg;
    msg << link << " is null.";

    throw ahl_robot::Exception("Robot::connectJointWithLink", msg.str());
  }

  joints_[joint]->setParentLink(links_[link]);
  links_[link]->addChildJoint(joints_[joint]);
}

void Robot::setBaseFrame(const std::string& name)
{
  Links::iterator it = links_.find(name);

  if(it == links_.end())
  {
    std::stringstream msg;
    msg << "Could not find " << name;

    throw ahl_robot::Exception("Robot::setBaseFrame", msg.str());
  }

  base_frame_ = name;
}

void Robot::setOperatedFrame(const std::string& name, const std::string& parent, const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy)
{
  Links::iterator it = links_.find(parent);
  if(it == links_.end())
  {
    std::stringstream msg;
    msg << "Could not find " << parent << " link";

    throw ahl_robot::Exception("Robot::setOperatedFrame", msg.str());
  }

  Eigen::Vector3d axis;
  axis << 0, 0, 1;

  JointPtr joint = JointPtr(
                     new Joint(
                       name, true,
                       xyz.coeff(0), xyz.coeff(1), xyz.coeff(2),
                       rpy.coeff(0), rpy.coeff(1), rpy.coeff(2),
                       axis,
                       0.0, 0.0, 0.0, 0.0
                     )
                   );

  this->addJoint(joint);
  this->connectJointWithParentLink(name, parent);

  operated_frame_ = name;
}

void Robot::setup()
{
  Links::iterator link_it;
  std::map<std::string, Eigen::Matrix4d> T_link;

  for(link_it = links_.begin(); link_it != links_.end(); ++link_it)
  {
    T_link[link_it->first] = link_it->second->getT();
  }

  for(link_it = links_.begin(); link_it != links_.end(); ++link_it)
  {
    LinkPtr link = link_it->second;

    JointPtr joint = link->getJoint();
    if(!joint)
      continue;

    LinkPtr parent_link = joint->getParentLink();
    if(!parent_link)
      continue;

    // world to parent link
    Eigen::Matrix4d Twp = T_link[parent_link->getName()];
    // parent link to world
    Eigen::Matrix4d Tpw;
    utils::calculateInverseTransformationMatrix(Twp, Tpw);
    // parent link to link
    Eigen::Matrix4d Tpl = Tpw * T_link[link->getName()];

    link->setT(Tpl);
  }

  Joints::iterator joint_it;
  std::map<std::string, Eigen::Matrix4d> T_joint;

  for(joint_it = joints_.begin(); joint_it != joints_.end(); ++joint_it)
  {
    T_joint[joint_it->first] = joint_it->second->getT();
  }

  for(joint_it = joints_.begin(); joint_it != joints_.end(); ++joint_it)
  {
    JointPtr joint = joint_it->second;

    LinkPtr link = joint->getLink();
    if(!link)
      continue;

    // parent link to link
    Eigen::Matrix4d Tpl = link->getT();
    // link to joint
    Eigen::Matrix4d Tlj = T_joint[joint->getName()];
    // parent link to joint
    Eigen::Matrix4d Tpj = Tpl * Tlj;

    joint->setT(Tpj);
  }

  //this->setupManipulator();
  //this->setupMobileBase();
}

const Joints& Robot::getJoints() const
{
  return joints_;
}

const Links& Robot::getLinks() const
{
  return links_;
}

const LinkPtr& Robot::getBaseLink() const
{
  Links::const_iterator it = links_.find(base_frame_);
  if(it == links_.end())
  {
    std::stringstream msg;
    msg << "Could not find " << base_frame_ << " link.";

    throw ahl_robot::Exception("Robot::getBaseLink", msg.str());
  }

  return it->second;
}

const ManipulatorPtr& Robot::getManipulator() const
{
  return manipulator_;
}

const MobileBasePtr& Robot::getMobileBase() const
{
  return mobile_base_;
}

void Robot::print()
{
  Joints::iterator joints_it;
  Links::iterator links_it;

  for(joints_it = joints_.begin(); joints_it != joints_.end(); ++joints_it)
  {
    joints_it->second->print();
  }

  for(links_it = links_.begin(); links_it != links_.end(); ++links_it)
  {
    links_it->second->print();
  }
}

void Robot::printNameList()
{
  std::map<std::string, JointPtr>::iterator joints_it;
  std::map<std::string, LinkPtr>::iterator links_it;

  std::cout << "<< Joint list >>" << std::endl;
  for(joints_it = joints_.begin(); joints_it != joints_.end(); ++joints_it)
  {
    std::cout << joints_it->first << std::endl;
  }

  std::cout << "<< Link list >>" << std::endl;
  for(links_it = links_.begin(); links_it != links_.end(); ++links_it)
  {
    std::cout << links_it->first << std::endl;
  }
}

void Robot::setupManipulator()
{
  JointPtr joint = joints_[operated_frame_];
  if(!joint)
  {
    std::stringstream msg;
    msg << "Could not find operated frame : " << operated_frame_;

    throw ahl_robot::Exception("Robot::setupManipulator", msg.str());
  }

  LinkPtr root_link = links_[base_frame_];
  if(!root_link)
  {
    std::stringstream msg;
    msg << "Could not find base frame : " << base_frame_;

    throw ahl_robot::Exception("Robot::setupManipulator", msg.str());
  }

  while(true)
  {
    if(!joint)
      break;

    manipulator_->addJoint(joint);

    LinkPtr parent_link = joint->getParentLink();
    if(!parent_link)
    {
      std::stringstream msg;
      msg << joint->getName() << " doesn't have parent link.";

      throw ahl_robot::Exception("Robot::setupManipulator", msg.str());
    }

    manipulator_->addLink(parent_link);

    if(parent_link->getName() == base_frame_)
    {
      break;
    }

    joint = parent_link->getJoint();
  }

  manipulator_->setup();
}

void Robot::setupMobileBase()
{

}
