#include "ahl_youbot_server/state/state.hpp"

using namespace ahl_youbot;

State::State()
{

}

bool State::callFloat(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_srvs::Float::Response& res)
{
  return false;
}

bool State::callSetJoint(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_srvs::SetJoint::Response& res)
{
  return false;
}

bool State::callJointSpaceControl(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_srvs::JointSpaceControl::Response& res)
{
  return false;
}

bool State::callTaskSpaceControl(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_srvs::TaskSpaceControl::Response& res)
{
  return false;
}

bool State::callTaskSpaceHybridControl(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_srvs::TaskSpaceHybridControl::Response& res)
{
  return false;
}
