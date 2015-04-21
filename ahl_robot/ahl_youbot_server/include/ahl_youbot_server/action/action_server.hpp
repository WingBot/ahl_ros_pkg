#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <boost/shared_ptr.hpp>
#include <actionlib/server/simple_action_server.h>
#include "ahl_youbot_server/ahl_robot_actions.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer();
  private:
    typedef actionlib::SimpleActionServer<ahl_robot_actions::FloatAction> FloatServer;
    typedef actionlib::SimpleActionServer<ahl_robot_actions::SetJointAction> SetJointServer;
    typedef actionlib::SimpleActionServer<ahl_robot_actions::JointSpaceControlAction> JointSpaceControlServer;
    typedef actionlib::SimpleActionServer<ahl_robot_actions::TaskSpaceControlAction> TaskSpaceControlServer;
    typedef actionlib::SimpleActionServer<ahl_robot_actions::TaskSpaceHybridControlAction> TaskSpaceHybridControlServer;
    typedef boost::shared_ptr<FloatServer> FloatServerPtr;
    typedef boost::shared_ptr<SetJointServer> SetJointServerPtr;
    typedef boost::shared_ptr<JointSpaceControlServer> JointSpaceControlServerPtr;
    typedef boost::shared_ptr<TaskSpaceControlServer> TaskSpaceControlServerPtr;
    typedef boost::shared_ptr<TaskSpaceHybridControlServer> TaskSpaceHybridControlServerPtr;

    FloatServerPtr float_;
    SetJointServerPtr set_joint_;
    JointSpaceControlServerPtr joint_space_control_;
    TaskSpaceControlServerPtr task_space_control_;
    TaskSpaceHybridControlServerPtr task_space_hybrid_control_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
