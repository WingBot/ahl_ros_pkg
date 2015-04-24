#include "ahl_youbot_server/action/float_action.hpp"

using namespace ahl_youbot;

FloatAction::FloatAction(const std::string& action_name)
  : Action(action_name)
{
  server_ = FloatServerPtr(
    new FloatServer(
      getNodeHandle(), getActionName(), boost::bind(&FloatAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void FloatAction::executeCB(const ahl_robot_actions::FloatGoalConstPtr& goal)
{

}

bool FloatAction::isActive()
{
  return server_->isActive();
}

bool FloatAction::isNewGoalAvailable()
{
  return false;
}

bool FloatAction::isPreemptRequested()
{
  return false;
}

void FloatAction::start()
{
  server_->start();
}

void FloatAction::shutdown()
{
  server_->shutdown();
}
