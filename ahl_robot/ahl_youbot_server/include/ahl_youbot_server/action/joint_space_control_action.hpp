#ifndef __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP
#define __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP

#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/youbot/youbot.hpp"

namespace ahl_youbot
{

  class JointSpaceControlAction : public Action
  {
  public:
    JointSpaceControlAction(const std::string& action_name, const YouBotPtr& youbot);

    virtual void execute(void* goal);

  private:
    YouBotPtr youbot_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP */
