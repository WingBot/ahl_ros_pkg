#ifndef __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP
#define __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP

#include <actionlib/server/simple_action_server.h>
#include <ahl_robot_actions/JointSpaceControlAction.h>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class JointSpaceControlAction : public Action
  {
  public:
    JointSpaceControlAction();

    virtual bool isActive();
    virtual bool isNewGoalAvailable();
    virtual bool isPreemptRequested();
    virtual void start();
    virtual void shutdown();

  private:
    void executeCB(const ahl_robot_actions::JointSpaceControlGoalConstPtr& goal);

    typedef actionlib::SimpleActionServer<ahl_robot_actions::JointSpaceControlAction> JointSpaceControlServer;
    typedef boost::shared_ptr<JointSpaceControlServer> JointSpaceControlServerPtr;

    static const std::string ACTION_NAME_;
    JointSpaceControlServerPtr server_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_JOINT_SPACE_CONTROL_ACTION_HPP */
