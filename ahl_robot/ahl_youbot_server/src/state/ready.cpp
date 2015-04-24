#include "ahl_youbot_server/state/ready.hpp"

using namespace ahl_youbot;

bool Ready::callFloat(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_srvs::Float::Response& res)
{
  ahl_robot_actions::FloatGoal goal;
  this->convertServiceToAction(req, goal);

  return true;
}

bool Ready::callSetJoint(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_srvs::SetJoint::Response& res)
{
  ahl_robot_actions::SetJointGoal goal;
  this->convertServiceToAction(req, goal);

  return true;
}

bool Ready::callJointSpaceControl(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_srvs::JointSpaceControl::Response& res)
{
  ahl_robot_actions::JointSpaceControlGoal goal;
  this->convertServiceToAction(req, goal);

  return true;
}

bool Ready::callTaskSpaceControl(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_srvs::TaskSpaceControl::Response& res)
{
  ahl_robot_actions::TaskSpaceControlGoal goal;
  this->convertServiceToAction(req, goal);

  return true;
}

bool Ready::callTaskSpaceHybridControl(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_srvs::TaskSpaceHybridControl::Response& res)
{
  ahl_robot_actions::TaskSpaceHybridControlGoal goal;
  this->convertServiceToAction(req, goal);

  return true;
}
