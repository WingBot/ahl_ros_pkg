#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/set_joint_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"
#include "ahl_youbot_server/interface/youbot_interface.hpp"
#include "ahl_youbot_server/interface/gazebo_interface.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer(bool use_real_robot)
{
  action_type_ = Action::FLOAT;
  ros::NodeHandle nh;

  if(use_real_robot)
  {
    interface_ = InterfacePtr(new YoubotInterface());
  }
  else
  {

  }

  timer_ = nh.createTimer(ros::Duration(1.0), &ActionServer::timerCB, this);
}

void ActionServer::timerCB(const ros::TimerEvent&)
{
  void* test;
  action_[action_type_]->execute(test);
}
