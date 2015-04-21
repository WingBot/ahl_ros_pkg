#include "ahl_youbot_server/action/float_action.hpp"

using namespace ahl_youbot;

const std::string FloatAction::ACTION_NAME_ = "youbot/float_action";

FloatAction::FloatAction()
{
  server_ = FloatServerPtr(
    new FloatServer(
      getNodeHandle(), ACTION_NAME_, boost::bind(&FloatAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void FloatAction::executeCB(const ahl_robot_actions::FloatGoalConstPtr& goal)
{

}

bool FloatAction::isActive()
{
  return false;
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
