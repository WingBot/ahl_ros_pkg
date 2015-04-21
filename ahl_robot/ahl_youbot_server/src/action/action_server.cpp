#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/set_joint_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer()
{
  action_[Action::FLOAT] = ActionPtr(new FloatAction());
  action_[Action::SET_JOINT] = ActionPtr(new SetJointAction());
  action_[Action::JOINT_SPACE_CONTROL] = ActionPtr(new JointSpaceControlAction());
  action_[Action::TASK_SPACE_CONTROL] = ActionPtr(new TaskSpaceControlAction());
  action_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionPtr(new TaskSpaceHybridControlAction());

  ROS_INFO_STREAM("start");
  start(Action::FLOAT);
  ROS_INFO_STREAM(isActive(Action::FLOAT));
  ROS_INFO_STREAM("shutdown");
  shutdown(Action::FLOAT);
  ROS_INFO_STREAM(isActive(Action::FLOAT));
}

void ActionServer::start(Action::Type type)
{
  action_[type]->start();
}

void ActionServer::cancel(Action::Type type)
{

}

void ActionServer::shutdown(Action::Type type)
{
  action_[type]->shutdown();
}

bool ActionServer::isActive(Action::Type type)
{
  action_[type]->isActive();
}
