#include "ahl_youbot_server/action/set_joint_action.hpp"

using namespace ahl_youbot;

SetJointAction::SetJointAction(const std::string& action_name, const ahl_robot::RobotPtr& robot)
  : Action(action_name), robot_(robot)
{
}

void SetJointAction::execute(void* goal)
{
  ROS_INFO_STREAM("SetJointAction");
}
