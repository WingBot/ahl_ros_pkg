#include "ahl_youbot_server/action/task_space_control_action.hpp"

using namespace ahl_youbot;

const std::string TaskSpaceControlAction::ACTION_NAME_ = "youbot/task_space_control_action";

TaskSpaceControlAction::TaskSpaceControlAction()
{
  server_ = TaskSpaceControlServerPtr(
    new TaskSpaceControlServer(
      getNodeHandle(), ACTION_NAME_, boost::bind(&TaskSpaceControlAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void TaskSpaceControlAction::executeCB(const ahl_robot_actions::TaskSpaceControlGoalConstPtr& goal)
{

}

bool TaskSpaceControlAction::isActive()
{
  return false;
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
