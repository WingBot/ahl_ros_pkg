#include "ahl_youbot_server/action/joint_space_control_action.hpp"

using namespace ahl_youbot;

JointSpaceControlAction::JointSpaceControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
}

void JointSpaceControlAction::execute(void* goal)
{
  ROS_INFO_STREAM("JointSpaceControlAction");
}
