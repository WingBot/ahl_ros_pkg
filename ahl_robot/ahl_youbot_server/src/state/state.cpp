#include "ahl_youbot_server/state/state.hpp"

using namespace ahl_youbot;

State::State(const ActionServerPtr& action_server)
  : action_server_(action_server)
{

}

bool State::callCancel(
  std_srvs::Empty::Request& req,
  std_srvs::Empty::Response& res)
{
  action_server_->cancel();
  return false;
}

bool State::callFloat(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_srvs::Float::Response& res)
{
  action_server_->cancel();
  return false;
}

bool State::callSetJoint(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_srvs::SetJoint::Response& res)
{
  action_server_->cancel();
  return false;
}

bool State::callJointSpaceControl(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_srvs::JointSpaceControl::Response& res)
{
  action_server_->cancel();
  return false;
}

bool State::callTaskSpaceControl(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_srvs::TaskSpaceControl::Response& res)
{
  action_server_->cancel();
  return false;
}

bool State::callTaskSpaceHybridControl(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_srvs::TaskSpaceHybridControl::Response& res)
{
  action_server_->cancel();
  return false;
}

void State::convertServiceToAction(
  ahl_robot_srvs::Float::Request& req,
  ahl_robot_actions::FloatGoal& goal)
{
  // Do nothing
}

void State::convertServiceToAction(
  ahl_robot_srvs::SetJoint::Request& req,
  ahl_robot_actions::SetJointGoal& goal)
{
  goal.qd.resize(req.qd.size());

  for(unsigned int i = 0; i < req.qd.size(); ++i)
  {
    goal.qd[i] = req.qd[i];
  }
}

void State::convertServiceToAction(
  ahl_robot_srvs::JointSpaceControl::Request& req,
  ahl_robot_actions::JointSpaceControlGoal& goal)
{
  goal.xpd = req.xpd;

  goal.xrd.resize(req.xrd.size());
  for(unsigned int i = 0; i < req.xrd.size(); ++i)
  {
    goal.xrd[i] = req.xrd[i];
  }
}

void State::convertServiceToAction(
  ahl_robot_srvs::TaskSpaceControl::Request& req,
  ahl_robot_actions::TaskSpaceControlGoal& goal)
{
  goal.xpd = req.xpd;

  goal.xrd.resize(req.xrd.size());
  for(unsigned int i = 0; i < req.xrd.size(); ++i)
  {
    goal.xrd[i] = req.xrd[i];
  }
}

void State::convertServiceToAction(
  ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
  ahl_robot_actions::TaskSpaceHybridControlGoal& goal)
{
  for(unsigned int i = 0; i < req.f_ori.size(); ++i)
  {
    goal.f_ori[i] = req.f_ori[i];
  }
  for(unsigned int i = 0; i < req.m_ori.size(); ++i)
  {
    goal.m_ori[i] = req.m_ori[i];
  }

  goal.xpd = req.xpd;
  goal.xrd.resize(req.xrd.size());

  for(unsigned int i = 0; i < req.f.size(); ++i)
  {
    goal.f[i] = req.f[i];
  }
  for(unsigned int i = 0; i < req.m.size(); ++i)
  {
    goal.m[i] = req.m[i];
  }

  goal.duration_f = req.duration_f;
  goal.duration_m = req.duration_m;
}
