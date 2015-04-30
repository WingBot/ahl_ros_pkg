#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/set_joint_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer(bool use_real_robot)
{
  action_type_ = Action::FLOAT;
/*
  action_[Action::FLOAT] = ActionPtr(
    new FloatAction("ahl_youbot/float", youbot_));
  action_[Action::SET_JOINT] = ActionPtr(
    new SetJointAction("ahl_youbot/set_joint", youbot_));
  action_[Action::JOINT_SPACE_CONTROL] = ActionPtr(
    new JointSpaceControlAction("ahl_youbot/joint_space_control", youbot_));
  action_[Action::TASK_SPACE_CONTROL] = ActionPtr(
    new TaskSpaceControlAction("ahl_youbot/task_space_control", youbot_));
  action_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionPtr(
    new TaskSpaceHybridControlAction("ahl_youbot/task_space_hybrid_control", youbot_));
*/
  ros::NodeHandle nh;

  timer_ = nh.createTimer(ros::Duration(1.0), &ActionServer::timerCB, this);
}

void ActionServer::timerCB(const ros::TimerEvent&)
{
  void* test;
  action_[action_type_]->execute(test);
}
