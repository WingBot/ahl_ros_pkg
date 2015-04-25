#include "ahl_youbot_server/action/task_space_control_action.hpp"

using namespace ahl_youbot;

TaskSpaceControlAction::TaskSpaceControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
  server_ = TaskSpaceControlServerPtr(
    new TaskSpaceControlServer(
      getNodeHandle(), getActionName(), boost::bind(&TaskSpaceControlAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void TaskSpaceControlAction::executeCB(const ahl_robot_actions::TaskSpaceControlGoalConstPtr& goal)
{

}

bool TaskSpaceControlAction::isActive()
{
  return server_->isActive();
}

bool TaskSpaceControlAction::isNewGoalAvailable()
{
  return false;
}

bool TaskSpaceControlAction::isPreemptRequested()
{
  return false;
}

void TaskSpaceControlAction::start()
{
  server_->start();
}

void TaskSpaceControlAction::shutdown()
{
  server_->shutdown();
}
