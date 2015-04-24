#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "ahl_youbot_server/ahl_robot_actions.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/action/action_client.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer();

    template<class ActionGoal>
    void sendGoal(Action::Type type, const ActionGoal& goal)
    {
      client_[type]->sendGoal(goal);
    }

    void start(Action::Type type);
    void preempt(Action::Type type);
    void cancel(Action::Type type);
    void cancel();
    void shutdown(Action::Type type);
    bool isActive(Action::Type type);

  private:
    typedef actionlib::SimpleActionClient<ahl_robot_actions::FloatAction> FloatClient;
    typedef actionlib::SimpleActionClient<ahl_robot_actions::SetJointAction> SetJointClient;
    typedef actionlib::SimpleActionClient<ahl_robot_actions::JointSpaceControlAction> JointSpaceControlClient;
    typedef actionlib::SimpleActionClient<ahl_robot_actions::TaskSpaceControlAction> TaskSpaceControlClient;
    typedef actionlib::SimpleActionClient<ahl_robot_actions::TaskSpaceHybridControlAction> TaskSpaceHybridControlClient;
    typedef boost::shared_ptr<FloatClient> FloatClientPtr;
    typedef boost::shared_ptr<SetJointClient> SetJointClientPtr;
    typedef boost::shared_ptr<JointSpaceControlClient> JointSpaceControlClientPtr;
    typedef boost::shared_ptr<TaskSpaceControlClient> TaskSpaceControlClientPtr;
    typedef boost::shared_ptr<TaskSpaceHybridControlClient> TaskSpaceHybridControlClientPtr;

    std::map<Action::Type, ActionPtr> action_;
    std::map<Action::Type, ros::Publisher> canceller_;

/*
    FloatClientPtr client_float_;
    SetJointClientPtr client_set_joint_;
    JointSpaceControlClientPtr client_joint_space_control_;
    TaskSpaceControlClientPtr client_task_space_control_;
    TaskSpaceHybridControlClientPtr client_task_space_hybrid_control_;
*/
    std::map<Action::Type, ActionClientBasePtr> client_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
