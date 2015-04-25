#include "ahl_youbot_server/action/joint_space_control_action.hpp"

using namespace ahl_youbot;

JointSpaceControlAction::JointSpaceControlAction(const std::string& action_name, const YouBotPtr& youbot)
  : youbot_(youbot), Action(action_name)
{
  server_ = JointSpaceControlServerPtr(
    new JointSpaceControlServer(
      getNodeHandle(), getActionName(), boost::bind(&JointSpaceControlAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void JointSpaceControlAction::executeCB(const ahl_robot_actions::JointSpaceControlGoalConstPtr& goal)
{

}

bool JointSpaceControlAction::isActive()
{
  return server_->isActive();
}

bool JointSpaceControlAction::isNewGoalAvailable()
{
  return false;
}

bool JointSpaceControlAction::isPreemptRequested()
{
  return false;
}

void JointSpaceControlAction::start()
{
  server_->start();
}

void JointSpaceControlAction::shutdown()
{
  server_->shutdown();
}
