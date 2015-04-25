#include "ahl_youbot_server/youbot/youbot.hpp"
#include "ahl_youbot_server/youbot/real_youbot_base.hpp"
#include "ahl_youbot_server/youbot/gazebo_youbot_base.hpp"
#include "ahl_youbot_server/youbot/real_youbot_manipulator.hpp"
#include "ahl_youbot_server/youbot/gazebo_youbot_manipulator.hpp"

using namespace ahl_youbot;

YouBot::YouBot(bool use_real_robot)
{
  if(use_real_robot)
  {
    base_ = AHLYouBotBasePtr(new RealYouBotBase());
    manipulator_ = AHLYouBotManipulatorPtr(new RealYouBotManipulator());
  }
  else
  {
    base_ = AHLYouBotBasePtr(new GazeboYouBotBase());
    manipulator_ = AHLYouBotManipulatorPtr(new GazeboYouBotManipulator());
  }
}
