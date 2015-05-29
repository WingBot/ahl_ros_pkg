#include "ahl_youbot_server/action/joint_space_control_action.hpp"

using namespace ahl_youbot;

JointSpaceControlAction::JointSpaceControlAction(const std::string& action_name, const ahl_robot::RobotPtr& robot)
  : Action(action_name), robot_(robot)
{
}

void JointSpaceControlAction::execute(void* goal)
{
  ROS_INFO_STREAM("JointSpaceControlAction");
}
