#include "ahl_youbot_sample/viewer/viewer.hpp"

using namespace ahl_youbot;

Viewer::Viewer()
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("ahl_youbot_sample");

  double duration;
  std::string cfg_youbot_base;
  std::string cfg_youbot_manipulator;

  local_nh.param<double>("tele_operation/duration", duration, 200);
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_base", cfg_youbot_base, "youbot-base");
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_manipulator", cfg_youbot_manipulator, "youbot-manipulator");

  timer_ = nh.createTimer(ros::Duration(duration), &Viewer::timerCB, this);

  base_ = YouBotBasePtr(
    new youbot::YouBotBase(cfg_youbot_base, YOUBOT_CONFIGURATIONS_DIR));
  base_->doJointCommutation();

  manipulator_ = YouBotManipulatorPtr(
    new youbot::YouBotManipulator(cfg_youbot_manipulator, YOUBOT_CONFIGURATIONS_DIR));
  manipulator_->doJointCommutation();
}

void Viewer::timerCB(const ros::TimerEvent&)
{



}
