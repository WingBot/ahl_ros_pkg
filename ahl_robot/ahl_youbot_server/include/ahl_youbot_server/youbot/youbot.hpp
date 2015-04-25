#ifndef __AHL_YOUBOT_SERVER_YOUBOT_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_HPP

#include "ahl_youbot_server/youbot/youbot_base.hpp"
#include "ahl_youbot_server/youbot/youbot_manipulator.hpp"

namespace ahl_youbot
{

  class YouBot
  {
  public:
    YouBot(bool use_real_robot);

    AHLYouBotBasePtr& base()
    {
      return base_;
    }

    AHLYouBotManipulatorPtr& manipulator()
    {
      return manipulator_;
    }

  private:
    AHLYouBotBasePtr base_;
    AHLYouBotManipulatorPtr manipulator_;
  };

  typedef boost::shared_ptr<YouBot> YouBotPtr;
};

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_HPP */
