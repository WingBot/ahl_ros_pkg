#ifndef __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP
#define __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP

#include <ahl_robot_srvs/Float.h>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/task/task.hpp>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class FloatAction : public Action
  {
  public:
    FloatAction(const std::string& action_name, const ahl_robot::RobotPtr& robot, const ahl_ctrl::RobotControllerPtr& controller);

    virtual void execute(void* goal);

  private:
    typedef ahl_robot_srvs::Float::Request FloatRequest;
    typedef boost::shared_ptr<FloatRequest> FloatRequestPtr;

    ahl_ctrl::RobotControllerPtr controller_;
    ahl_ctrl::TaskPtr task_;
    ahl_robot::RobotPtr robot_;
    FloatRequestPtr req_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP  */
