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

void Robot::computeBasicJacobian(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::BasicJacobian", msg.str());
  }

  mnp_[mnp_name]->computeBasicJacobian();
}

void Robot::computeMassMatrix(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::computeMassMatrix", msg.str());
  }

  mnp_[mnp_name]->computeMassMatrix();
}

bool Robot::reached(const std::string& mnp_name, const Eigen::VectorXd& qd, double threshold)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::reached", msg.str());
  }

  return mnp_[mnp_name]->reached(qd, threshold);
}

void Robot::add(const ManipulatorPtr& mnp)
{
  mnp_[mnp->name] = mnp;
  mnp_name_.push_back(mnp->name);
}

const Eigen::MatrixXd& Robot::getBasicJacobian(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  std::string link_name = mnp_[mnp_name]->link[mnp_[mnp_name]->link.size() - 1]->name;
  return this->getBasicJacobian(mnp_name, link_name);
}

const Eigen::MatrixXd& Robot::getBasicJacobian(const std::string& mnp_name, const std::string& link_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  if(mnp_[mnp_name]->name_to_idx.find(link_name) == mnp_[mnp_name]->name_to_idx.end())
  {
    std::stringstream msg;
    msg << "Could not find name_to_idx." << std::endl
        << "  Manipulator : " << mnp_name << std::endl
        << "  Link        : " << link_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getBasicJacobian", msg.str());
  }

  int idx = mnp_[mnp_name]->name_to_idx[link_name];
  return mnp_[mnp_name]->J0[idx];
}

const Eigen::MatrixXd& Robot::getMassMatrix(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getMassMatrix", msg.str());
  }

  return mnp_[mnp_name]->M;
}

const Eigen::MatrixXd& Robot::getMassMatrixInv(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getMassMatrixInv", msg.str());
  }

  return mnp_[mnp_name]->M_inv;
}

const Eigen::VectorXd& Robot::getJointPosition(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getJointPosition", msg.str());
  }

  return mnp_[mnp_name]->q;
}

const Eigen::VectorXd& Robot::getJointVelocity(const std::string& mnp_name)
{
  if(mnp_.find(mnp_name) == mnp_.end())
  {
    std::stringstream msg;
    msg << "Could not find manipulator : " << mnp_name;
    throw ahl_robot::Exception("ahl_robot::Robot::getJointVelocity", msg.str());
  }

  return mnp_[mnp_name]->dq;
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
