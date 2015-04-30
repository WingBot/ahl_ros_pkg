#include "ahl_youbot_server/action/float_action.hpp"

using namespace ahl_youbot;

FloatAction::FloatAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
  req_ = FloatRequestPtr(new FloatRequest());
}

void FloatAction::execute(void* goal)
{
  req_.get();

  ROS_INFO_STREAM("FloatAction");
}
