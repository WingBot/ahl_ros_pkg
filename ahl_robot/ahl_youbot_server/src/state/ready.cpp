#include "ahl_youbot_server/state/ready.hpp"

using namespace ahl_youbot;

bool Ready::callFloat(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_srvs::Float::Response& res)
{
  return true;
}

bool Ready::callSetJoint(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_srvs::SetJoint::Response& res)
{
  return true;
}

bool Ready::callJointSpaceControl(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_srvs::JointSpaceControl::Response& res)
{
  return true;
}

bool Ready::callTaskSpaceControl(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_srvs::TaskSpaceControl::Response& res)
{
  return true;
}

bool Ready::callTaskSpaceHybridControl(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_srvs::TaskSpaceHybridControl::Response& res)
{
  return true;
}
