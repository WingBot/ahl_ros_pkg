#include "ahl_youbot_server/action/joint_space_control_action.hpp"

using namespace ahl_youbot;

const std::string JointSpaceControlAction::ACTION_NAME_ = "youbot/joint_space_control_action";

JointSpaceControlAction::JointSpaceControlAction()
{
  server_ = JointSpaceControlServerPtr(
    new JointSpaceControlServer(
      getNodeHandle(), ACTION_NAME_, boost::bind(&JointSpaceControlAction::executeCB, this, _1), false
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
