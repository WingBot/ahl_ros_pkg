#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/float_action.hpp"
#include "ahl_youbot_server/action/set_joint_action.hpp"
#include "ahl_youbot_server/action/joint_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_control_action.hpp"
#include "ahl_youbot_server/action/task_space_hybrid_control_action.hpp"
#include "ahl_youbot_server/interface/youbot_interface.hpp"
#include "ahl_youbot_server/interface/gazebo_interface.hpp"

using namespace ahl_youbot;

ActionServer::ActionServer(
  const std::string& robot_name, const std::string& mnp_name, const std::string& yaml,
  double period, double servo_period, bool use_real_robot)
  : updated_model_(false)
{
  action_type_ = Action::FLOAT;

  // Initialize interface
  if(use_real_robot)
  {
    interface_ = InterfacePtr(new YoubotInterface());
  }
  else
  {
    std::vector<std::string> joint_list;
    joint_list.push_back("youbot::arm_joint_1");
    joint_list.push_back("youbot::arm_joint_2");
    joint_list.push_back("youbot::arm_joint_3");
    joint_list.push_back("youbot::arm_joint_4");
    joint_list.push_back("youbot::arm_joint_5");
    interface_ = InterfacePtr(new GazeboInterface(joint_list, servo_period));
  }

  // Initialize robot
  using namespace ahl_robot;
  robot_ = RobotPtr(new Robot(robot_name));
  ParserPtr parser = ParserPtr(new Parser());
  parser->load(yaml, robot_);
  robot_->getJointStates(mnp_name, q_);
  dq_ = Eigen::VectorXd::Zero(q_.rows());

  // Initialize action
  action_[Action::FLOAT] = ActionPtr(
    new FloatAction("float", robot_));
  action_[Action::SET_JOINT] = ActionPtr(
    new SetJointAction("set_joint_action", robot_));
  action_[Action::JOINT_SPACE_CONTROL] = ActionPtr(
    new JointSpaceControlAction("joint_space_control", robot_));
  action_[Action::TASK_SPACE_CONTROL] = ActionPtr(
    new TaskSpaceControlAction("task_space_control", robot_));
  action_[Action::TASK_SPACE_HYBRID_CONTROL] = ActionPtr(
    new TaskSpaceHybridControlAction("task_space_hybrid_control", robot_));

  // Initialize 2 timer loops
  ros::NodeHandle nh;

  timer_ = nh.createTimer(
    ros::Duration(period), &ActionServer::timerCB, this);
  servo_timer_ = nh.createTimer(
    ros::Duration(servo_period), &ActionServer::servoTimerCB, this);
}

void ActionServer::start()
{
  boost::mutex::scoped_lock lock(mutex_);
  timer_.start();
  servo_timer_.start();
}

void ActionServer::stop()
{
  boost::mutex::scoped_lock lock(mutex_);
  timer_.stop();
  servo_timer_.stop();
}

void ActionServer::timerCB(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(mutex_);
  // update robot model
  if(interface_->getJointStates(q_))
  {
    updated_model_ = true;
  }
  else
  {
    updated_model_ = false;
  }
}

void ActionServer::servoTimerCB(const ros::TimerEvent&)
{
  boost::mutex::scoped_lock lock(mutex_);
  if(!updated_model_)
    return;

  void* test;
  action_[action_type_]->execute(test);
}
