#ifndef __AHL_YOUBOT_SERVER_TASK_SPACE_HYBRID_CONTROL_ACTION_HPP
#define __AHL_YOUBOT_SERVER_TASK_SPACE_HYBRID_CONTROL_ACTION_HPP

#include <actionlib/server/simple_action_server.h>
#include <ahl_robot_actions/TaskSpaceHybridControlAction.h>
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/youbot/youbot.hpp"

namespace ahl_youbot
{

  class TaskSpaceHybridControlAction : public Action
  {
  public:
    TaskSpaceHybridControlAction(const std::string& action_name, const YouBotPtr& youbot);

    virtual bool isActive();
    virtual bool isNewGoalAvailable();
    virtual bool isPreemptRequested();
    virtual void start();
    virtual void shutdown();

  private:
    void executeCB(const ahl_robot_actions::TaskSpaceHybridControlGoalConstPtr& goal);

    typedef actionlib::SimpleActionServer<ahl_robot_actions::TaskSpaceHybridControlAction> TaskSpaceHybridControlServer;
    typedef boost::shared_ptr<TaskSpaceHybridControlServer> TaskSpaceHybridControlServerPtr;

    TaskSpaceHybridControlServerPtr server_;
    YouBotPtr youbot_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_TASK_SPACE_HYBRID_CONTROL_ACTION_HPP */
