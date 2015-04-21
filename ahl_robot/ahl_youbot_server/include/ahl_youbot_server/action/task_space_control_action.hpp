#ifndef __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP
#define __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP

#include <actionlib/server/simple_action_server.h>
#include <ahl_robot_actions/TaskSpaceControlAction.h>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class TaskSpaceControlAction : public Action
  {
  public:
    TaskSpaceControlAction();

    virtual bool isActive();
    virtual bool isNewGoalAvailable();
    virtual bool isPreemptRequested();
    virtual void start();
    virtual void shutdown();

  private:
    void executeCB(const ahl_robot_actions::TaskSpaceControlGoalConstPtr& goal);

    typedef actionlib::SimpleActionServer<ahl_robot_actions::TaskSpaceControlAction> TaskSpaceControlServer;
    typedef boost::shared_ptr<TaskSpaceControlServer> TaskSpaceControlServerPtr;

    static const std::string ACTION_NAME_;
    TaskSpaceControlServerPtr server_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_TASK_SPACE_CONTROL_ACTION_HPP */
