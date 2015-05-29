#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/robot.hpp"

using namespace ahl_robot;

void Robot::update(const std::string& mnp_name, const Eigen::VectorXd& q)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::update", msg.str());
  }

  mnp_[mnp_name]->update(q);
}

void Robot::add(const ManipulatorPtr& mnp)
{
  mnp_[mnp->name] = mnp;
  mnp_name_.push_back(mnp->name);
}

void Robot::getJointStates(const std::string& mnp_name, Eigen::VectorXd& q)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    return;
  }

  q.resize(mnp_[mnp_name]->q.rows());
  q = mnp_[mnp_name]->q;
}

unsigned int Robot::getDOF(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getDOF", msg.str());
  }

  return mnp_[mnp_name]->dof;
}
