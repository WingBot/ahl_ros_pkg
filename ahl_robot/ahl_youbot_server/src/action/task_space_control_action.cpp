#include "ahl_youbot_server/action/task_space_control_action.hpp"

using namespace ahl_youbot;

TaskSpaceControlAction::TaskSpaceControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
}

void TaskSpaceControlAction::execute(void* goal)
{
  ROS_INFO_STREAM("TaskSpaceControlAction");
}
