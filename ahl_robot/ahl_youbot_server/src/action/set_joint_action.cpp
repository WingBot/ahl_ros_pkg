#include "ahl_youbot_server/action/set_joint_action.hpp"

using namespace ahl_youbot;

const std::string SetJointAction::ACTION_NAME_ = "youbot/set_joint_action";

SetJointAction::SetJointAction()
{
  server_ = SetJointServerPtr(
    new SetJointServer(
      getNodeHandle(), ACTION_NAME_, boost::bind(&SetJointAction::executeCB, this, _1), false
    )
  );

  server_->start();
}

void SetJointAction::executeCB(const ahl_robot_actions::SetJointGoalConstPtr& goal)
{

}

bool SetJointAction::isActive()
{
  return server_->isActive();
}

bool SetJointAction::isNewGoalAvailable()
{
  return false;
}

bool SetJointAction::isPreemptRequested()
{
  return false;
}

void SetJointAction::start()
{
  server_->start();
}

void SetJointAction::shutdown()
{
  server_->shutdown();
}
