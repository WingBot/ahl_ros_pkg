#include <ahl_robot/robot/manipulator.hpp>
#include <ahl_robot_controller/task/joint_control.hpp>
#include "ahl_youbot_server/action/float_action.hpp"

using namespace ahl_youbot;
using namespace ahl_robot;

FloatAction::FloatAction(const std::string& action_name, const ahl_robot::RobotPtr& robot, const ahl_ctrl::RobotControllerPtr& controller)
  : Action(action_name), robot_(robot)
{
  req_ = FloatRequestPtr(new FloatRequest());
  controller_ = controller;
  task_ = ahl_ctrl::TaskPtr(new ahl_ctrl::JointControl(robot->getManipulator("mnp"), 0));
}

void FloatAction::execute(void* goal)
{
  ManipulatorPtr mnp = robot_->getManipulator("mnp");


  //ROS_INFO_STREAM("FloatAction");
}
