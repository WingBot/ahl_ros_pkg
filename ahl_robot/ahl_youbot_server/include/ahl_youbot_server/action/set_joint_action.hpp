#ifndef __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP
#define __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP

#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class SetJointAction : public Action
  {
  public:
    SetJointAction(const std::string& action_name, const ahl_robot::RobotPtr& robot);

    virtual void execute(void* goal);

  private:
    ahl_robot::RobotPtr robot_;
 };

}

#endif /* __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP */
