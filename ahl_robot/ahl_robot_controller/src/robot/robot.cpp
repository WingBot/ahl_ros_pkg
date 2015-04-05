#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/robot/robot.hpp"

using namespace ahl_robot;

Robot::Robot(const std::string& base_link_name)
  : is_static_(false), base_link_name_(base_link_name)
{

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

void Robot::addJoint(const std::string& name, const JointPtr& joint)
{
  if(!joint)
  {
    std::stringstream msg;
    msg << "Could not add joint \"" << name << "\".";

    throw ahl_robot::Exception("ahl_robot::Robot::addJoint", msg.str());
  }

  joints_[name] = joint;
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

void Robot::addLink(const std::string& name, const LinkPtr& link)
{
  if(!link)
  {
    std::stringstream msg;
    msg << "Could not add link \"" << name << "\".";

    throw ahl_robot::Exception("ahl_robot::Robot::addLink", msg.str());
  }

  links_[name] = link;
}

void Robot::setRootLink(const LinkPtr& root_link)
{
  root_link_ = root_link;
}

void Robot::specifyEndEffector(const std::string& name)
{
  ee_names_.push_back(name);
}

const Joints& Robot::getJoints() const
{
  return joints_;
}

const Links& Robot::getLinks() const
{
  return links_;
}

const LinkPtr& Robot::getRootLink() const
{
  return root_link_;
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
