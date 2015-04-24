#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/set_joint_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer()
{
  action_[Action::FLOAT] = ActionPtr(
    new FloatAction("youbot/float"));
  action_[Action::SET_JOINT] = ActionPtr(
    new SetJointAction("youbot/set_joint"));
  action_[Action::JOINT_SPACE_CONTROL] = ActionPtr(
    new JointSpaceControlAction("youbot/joint_space_control"));
  action_[Action::TASK_SPACE_CONTROL] = ActionPtr(
    new TaskSpaceControlAction("youbot/task_space_control"));
  action_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionPtr(
    new TaskSpaceHybridControlAction("youbot/task_space_hybrid_control"));

  ros::NodeHandle nh;
  const int queue_size = 1;
  canceller_[Action::FLOAT] = nh.advertise<actionlib_msgs::GoalID>(
    action_[Action::FLOAT]->getActionName(), queue_size);
  canceller_[Action::SET_JOINT] = nh.advertise<actionlib_msgs::GoalID>(
    action_[Action::SET_JOINT]->getActionName(), queue_size);
  canceller_[Action::JOINT_SPACE_CONTROL] = nh.advertise<actionlib_msgs::GoalID>(
    action_[Action::JOINT_SPACE_CONTROL]->getActionName(), queue_size);
  canceller_[Action::TASK_SPACE_CONTROL] = nh.advertise<actionlib_msgs::GoalID>(
    action_[Action::TASK_SPACE_CONTROL]->getActionName(), queue_size);
  canceller_[Action::TASK_SPACE_HYBRID_CONTROL] = nh.advertise<actionlib_msgs::GoalID>(
    action_[Action::TASK_SPACE_HYBRID_CONTROL]->getActionName(), queue_size);

/*  client_float_ = FloatClientPtr(
    new FloatClient(
      action_[Action::FLOAT]->getActionName(), true));
  client_set_joint_ = SetJointClientPtr(
    new SetJointClient(
      action_[Action::SET_JOINT]->getActionName(), true));
  client_joint_space_control_ = JointSpaceControlClientPtr(
    new JointSpaceControlClient(
      action_[Action::JOINT_SPACE_CONTROL]->getActionName(), true));
  client_task_space_control_ = TaskSpaceControlClientPtr(
    new TaskSpaceControlClient(
      action_[Action::TASK_SPACE_CONTROL]->getActionName(), true));
  client_task_space_hybrid_control_ = TaskSpaceHybridControlClientPtr(
    new TaskSpaceHybridControlClient(
      action_[Action::TASK_SPACE_HYBRID_CONTROL]->getActionName(), true));
*/

  client_[Action::FLOAT] = ActionClientBasePtr(
    new ActionClient<ahl_robot_actions::FloatAction,
                     ahl_robot_actions::FloatGoal>(
      action_[Action::FLOAT]->getActionName())
  );
  client_[Action::SET_JOINT] = ActionClientBasePtr(
    new ActionClient<ahl_robot_actions::SetJointAction,
                     ahl_robot_actions::SetJointGoal>(
      action_[Action::SET_JOINT]->getActionName())
  );
  client_[Action::JOINT_SPACE_CONTROL] = ActionClientBasePtr(
    new ActionClient<ahl_robot_actions::JointSpaceControlAction,
                     ahl_robot_actions::JointSpaceControlGoal>(
      action_[Action::JOINT_SPACE_CONTROL]->getActionName())
  );
  client_[Action::TASK_SPACE_CONTROL] = ActionClientBasePtr(
    new ActionClient<ahl_robot_actions::TaskSpaceControlAction,
                     ahl_robot_actions::TaskSpaceControlGoal>(
      action_[Action::TASK_SPACE_CONTROL]->getActionName())
  );
  client_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionClientBasePtr(
    new ActionClient<ahl_robot_actions::TaskSpaceHybridControlAction,
                     ahl_robot_actions::TaskSpaceHybridControlGoal>(
      action_[Action::TASK_SPACE_HYBRID_CONTROL]->getActionName())
  );
}

void ActionServer::start(Action::Type type)
{
  action_[type]->start();
}

void ActionServer::preempt(Action::Type type)
{

}

void ActionServer::cancel(Action::Type type)
{
  actionlib_msgs::GoalID empty;
  canceller_[type].publish(empty);
}

void ActionServer::cancel()
{
  std::map<Action::Type, ros::Publisher>::iterator it;

  for(it = canceller_.begin(); it != canceller_.end(); ++it)
  {
    actionlib_msgs::GoalID empty;
    it->second.publish(empty);
  }
}

void ActionServer::shutdown(Action::Type type)
{
  action_[type]->shutdown();
}

bool ActionServer::isActive(Action::Type type)
{
  action_[type]->isActive();
}

