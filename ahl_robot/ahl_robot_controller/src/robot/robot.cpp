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
