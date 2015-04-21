#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

const std::string TaskSpaceHybridControlAction::ACTION_NAME_ = "youbot/task_space_hybrid_control_action";

TaskSpaceHybridControlAction::TaskSpaceHybridControlAction()
{
  server_ = TaskSpaceHybridControlServerPtr(
    new TaskSpaceHybridControlServer(
      getNodeHandle(), ACTION_NAME_, boost::bind(&TaskSpaceHybridControlAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void TaskSpaceHybridControlAction::executeCB(const ahl_robot_actions::TaskSpaceHybridControlGoalConstPtr& goal)
{

}

bool TaskSpaceHybridControlAction::isActive()
{
  return false;
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
