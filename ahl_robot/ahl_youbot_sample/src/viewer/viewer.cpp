#include "ahl_youbot_sample/viewer/viewer.hpp"
#include "ahl_youbot_sample/exception.hpp"

using namespace ahl_youbot;

Viewer::Viewer()
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("ahl_youbot_sample");

  double duration;
  std::string cfg_youbot_base;
  std::string cfg_youbot_manipulator;

  local_nh.param<double>("tele_operation/duration", duration, 0.5);
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
  base_->getBaseVelocity(vx_, vy_, vr_);
  manipulator_->getJointData(q_);
  manipulator_->getJointData(dq_);
  manipulator_->getJointData(tau_);

  std::cout << "Base : " << std::endl
            << "(vx, vy, vr) = ("
            << vx_.value() << ", "
            << vy_.value() << ", "
            << vr_.value() << ")" << std::endl;

  const int arm_dof = 5;

  if(q_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of q is wrong. It should be 5." << std::endl
        << "  size : " << q_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }
  else if(dq_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of dq is wrong. It should be 5." << std::endl
        << "  size : " << dq_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }
  else if(tau_.size() != arm_dof)
  {
    std::stringstream msg;
    msg << "The size of tau is wrong. It should be 5." << std::endl
        << "  size : " << tau_.size();

    throw ahl_youbot::Exception("ahl_youbot::Viewer::timerCB", msg.str());
  }

  std::cout << "Manipulator : " << std::endl
            << "(q1, q2, q3, q4, q5) = ("
            << q_[0].angle.value() << ", "
            << q_[1].angle.value() << ", "
            << q_[2].angle.value() << ", "
            << q_[3].angle.value() << ", "
            << q_[4].angle.value() << ")" << std::endl
            << "(dq1, dq2, dq3, dq4, dq5) = ("
            << dq_[0].angularVelocity.value() << ", "
            << dq_[1].angularVelocity.value() << ", "
            << dq_[2].angularVelocity.value() << ", "
            << dq_[3].angularVelocity.value() << ", "
            << dq_[4].angularVelocity.value() << ")" << std::endl
            << "(tau1, tau2, tau3, tau4, tau5) = ("
            << tau_[0].torque.value() << ", "
            << tau_[1].torque.value() << ", "
            << tau_[2].torque.value() << ", "
            << tau_[3].torque.value() << ", "
            << tau_[4].torque.value() << ")" << std::endl;
}
