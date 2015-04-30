#include "ahl_youbot_server/action/set_joint_action.hpp"

using namespace ahl_youbot;

SetJointAction::SetJointAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
}

void SetJointAction::execute(void* goal)
{
  ROS_INFO_STREAM("SetJointAction");
}
