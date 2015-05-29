#ifndef __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP
#define __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP

#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class TaskSpaceControlAction : public Action
  {
  public:
    TaskSpaceControlAction(const std::string& action_name, const ahl_robot::RobotPtr& robot);

    virtual void execute(void* goal);

  private:
    ahl_robot::RobotPtr robot_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP */
