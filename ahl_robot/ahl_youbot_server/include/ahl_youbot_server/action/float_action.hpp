#ifndef __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP
#define __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP

#include <ahl_robot_srvs/Float.h>
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/youbot/youbot.hpp"

namespace ahl_youbot
{

  class FloatAction : public Action
  {
  public:
    FloatAction(const std::string& action_name, const YouBotPtr& youbot);

    virtual void execute(void* goal);

  private:
    typedef ahl_robot_srvs::Float::Request FloatRequest;
    typedef boost::shared_ptr<FloatRequest> FloatRequestPtr;

    YouBotPtr youbot_;
    FloatRequestPtr req_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP  */
