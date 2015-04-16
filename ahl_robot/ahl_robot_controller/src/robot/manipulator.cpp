#include <algorithm>
#include <iterator>
#include "ahl_robot_controller/utils/math.hpp"
#include "ahl_robot_controller/robot/manipulator.hpp"

using namespace ahl_robot;

void Manipulator::addLink(const LinkPtr& link)
{
  links_.push_back(link);
}

void Manipulator::addJoint(const JointPtr& joint)
{
  joints_.push_back(joint);
}

void Manipulator::setup()
{
  std::vector<LinkPtr> tmp_links = links_;
  links_.clear();

  for(unsigned int i = 0; i < tmp_links.size(); ++i)
  {
    links_.push_back(tmp_links[tmp_links.size() - 1 - i]);
  }

  std::vector<JointPtr> tmp_joints = joints_;
  joints_.clear();

  for(unsigned int i = 0; i < tmp_joints.size(); ++i)
  {
    joints_.push_back(tmp_joints[tmp_joints.size() - 1 - i]);
  }
}

void Manipulator::print()
{
  for(unsigned int i = 0; i < links_.size(); ++i)
  {
    std::cout << links_[i]->getName() << std::endl;
  }
}
