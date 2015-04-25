#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

TaskSpaceHybridControlAction::TaskSpaceHybridControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
  server_ = TaskSpaceHybridControlServerPtr(
    new TaskSpaceHybridControlServer(
      getNodeHandle(), getActionName(), boost::bind(&TaskSpaceHybridControlAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void TaskSpaceHybridControlAction::executeCB(const ahl_robot_actions::TaskSpaceHybridControlGoalConstPtr& goal)
{

}

bool TaskSpaceHybridControlAction::isActive()
{
  return server_->isActive();
}

bool TaskSpaceHybridControlAction::isNewGoalAvailable()
{
  return false;
}

bool TaskSpaceHybridControlAction::isPreemptRequested()
{
  return false;
}

void TaskSpaceHybridControlAction::start()
{
  server_->start();
}

void TaskSpaceHybridControlAction::shutdown()
{
  server_->shutdown();
}
