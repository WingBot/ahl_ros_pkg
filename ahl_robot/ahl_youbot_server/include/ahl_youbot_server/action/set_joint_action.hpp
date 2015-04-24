#ifndef __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP
#define __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP

#include <actionlib/server/simple_action_server.h>
#include <ahl_robot_actions/SetJointAction.h>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class SetJointAction : public Action
  {
  public:
    SetJointAction(const std::string& action_name);

    virtual bool isActive();
    virtual bool isNewGoalAvailable();
    virtual bool isPreemptRequested();
    virtual void start();
    virtual void shutdown();

  private:
    void executeCB(const ahl_robot_actions::SetJointGoalConstPtr& goal);

    typedef actionlib::SimpleActionServer<ahl_robot_actions::SetJointAction> SetJointServer;
    typedef boost::shared_ptr<SetJointServer> SetJointServerPtr;

    SetJointServerPtr server_;
 };

}

#endif /* __AHL_YOUBOT_SERVER_SET_JOINT_ACTION_HPP */
