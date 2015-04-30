#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

TaskSpaceHybridControlAction::TaskSpaceHybridControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
}

void TaskSpaceHybridControlAction::execute(void* goal)
{
  ROS_INFO_STREAM("TaskSpaceHybridControlAction");
}
