#ifndef __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP
#define __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP

#include <actionlib/server/simple_action_server.h>
#include <ahl_robot_actions/FloatAction.h>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class FloatAction : public Action
  {
  public:
    FloatAction(const std::string& action_name);

    virtual bool isActive();
    virtual bool isNewGoalAvailable();
    virtual bool isPreemptRequested();
    virtual void start();
    virtual void shutdown();

  private:
    void executeCB(const ahl_robot_actions::FloatGoalConstPtr& goal);

    typedef actionlib::SimpleActionServer<ahl_robot_actions::FloatAction> FloatServer;
    typedef boost::shared_ptr<FloatServer> FloatServerPtr;

    FloatServerPtr server_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_FLOAT_ACTION_HPP  */
