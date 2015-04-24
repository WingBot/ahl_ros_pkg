#include "ahl_youbot_sample/exception.hpp"
#include "ahl_youbot_sample/tele_operation/tele_operation.hpp"

using namespace ahl_youbot;

TeleOperation::TeleOperation()
  : initialized_(false), running_(false)
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("ahl_youbot_sample");

  std::string cfg_youbot_base;
  std::string cfg_youbot_manipulator;

  local_nh.param<double>("tele_operation/duration", duration_, 0.25);
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_base", cfg_youbot_base, "youbot-base");
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_manipulator", cfg_youbot_manipulator, "youbot-manipulator");
  local_nh.param<double>("tele_operation/vx", vx_, 0.05);
  local_nh.param<double>("tele_operation/vy", vy_, 0.05);
  local_nh.param<double>("tele_operation/vr", vr_, 0.2);
  local_nh.param<double>("tele_operation/qd", qd_, 0.2);

  base_ = YouBotBasePtr(
    new youbot::YouBotBase(cfg_youbot_base, YOUBOT_CONFIGURATIONS_DIR));

  manipulator_ = YouBotManipulatorPtr(
    new youbot::YouBotManipulator(cfg_youbot_manipulator, YOUBOT_CONFIGURATIONS_DIR));
}

TeleOperation::~TeleOperation()
{
  refresh();
  endwin();
}

void TeleOperation::run()
{
  if(!initialized_)
  {
    this->init();
  }

  while(ros::ok() && running_)
  {
    this->control();

    std::cout << "(vx, vy, vr) = ("
              << dvx_ << ", " << dvy_ << ", " << dvr_ << ")\r" << std::endl;
    refresh();

    ros::Duration(duration_).sleep();
  }
}

void TeleOperation::init()
{
  base_->doJointCommutation();
  manipulator_->doJointCommutation();
  manipulator_->calibrateManipulator(true);
  manipulator_->calibrateGripper(true);

  static_cast<void>(initscr()); // initialize the curses library
  keypad(stdscr, TRUE); // enable keyboard mapping
  static_cast<void>(nonl()); // tell curses not to do NL->CR/NL on output
  static_cast<void>(cbreak()); // take input chars one at a time, no wait for \n

  def_prog_mode();
  refresh();

  dqd_.resize(manipulator_->getNumberJoints());
  if(dqd_.size() != 5)
  {
    std::stringstream msg;
    msg << "Wrong number of joints. It should be 5." << std::endl
        << "  number of joint : " << dqd_.size();

    throw ahl_youbot::Exception("ahl_youbot::TeleOperation::init", msg.str());
  }

  this->printHotKeys();
  refresh();

  initialized_ = true;
  running_ = true;
}

void TeleOperation::printHotKeys()
{
  std::cout << "up = drive forward\r" << std::endl
            << "down = drive backward\r" << std::endl
            << "left = drive left\r" << std::endl
            << "right = drive right\r" << std::endl
            << "x = turn right\r" << std::endl
            << "z = turn left\r" << std::endl
            << "1 = drive 1st joint of arm clockwise\r" << std::endl
            << "q = drive 1st joint of arm counterclockwise\r" << std::endl
            << "2 = drive 2nd joint of arm clockwise\r" << std::endl
            << "w = drive 2nd joint of arm counterclockwise\r" << std::endl
            << "3 = drive 3rd joint of arm clockwise\r" << std::endl
            << "e = drive 3rd joint of arm counterclockwise\r" << std::endl
            << "4 = drive 4th joint of arm clockwise\r" << std::endl
            << "r = drive 4th joint of arm counterclockwise\r" << std::endl
            << "5 = drive 5th joint of arm clockwise\r" << std::endl
            << "t = drive 5th joint of arm counterclockwise\r" << std::endl
            << "6 = close gripper\r" << std::endl
            << "y = open gripper\r" << std::endl
            << "Ctrl + C = shutdown the process\r" << std::endl;
}

void TeleOperation::control()
{
  std::cout << "*** WARNING *** DO NOT HOLD ANY KEYS ***\r" << std::endl;
  std::cout << "Press 's' or 'ESC' in order to stop the robot\r" << std::endl;

  int ch = getch();

  if(ch == 's' || ch == 27) // ESC == 27
  {
    this->stop();
  }
  else if(ch == KEY_UP || ch == KEY_DOWN || ch == KEY_LEFT || ch == KEY_RIGHT || ch == 'z' || ch == 'x')
  {
    this->controlBase(ch);
  }
  else if(ch == '6' || ch == 'y')
  {
    this->controlGripper(ch);
  }
  else
  {
    this->controlManipulator(ch);
  }
}

void TeleOperation::controlBase(int ch)
{
  dvx_ = 0 * meter_per_second;
  dvy_ = 0 * meter_per_second;
  dvr_ = 0 * radian_per_second;

  switch(ch)
  {
  case KEY_UP:
    dvx_ = vx_ * meter_per_second;
    break;
  case KEY_DOWN:
    dvx_ = -vx_ * meter_per_second;
    break;
  case KEY_LEFT:
    dvy_ = vy_ * meter_per_second;
    break;
  case KEY_RIGHT:
    dvy_ = -vy_ * meter_per_second;
    break;
  case 'z':
    dvr_ = vr_ * radian_per_second;
    break;
  case 'x':
    dvr_ = -vr_ * radian_per_second;
    break;
  default:
    break;
  };

  base_->setBaseVelocity(dvx_, dvy_, dvr_);
}

void TeleOperation::controlGripper(int ch)
{
  switch(ch)
  {
  case '6':
    manipulator_->getArmGripper().open();
    break;
  case 'y':
    manipulator_->getArmGripper().close();
    break;
  };
}

void TeleOperation::controlManipulator(int ch)
{
  for(unsigned int i = 0; i < dqd_.size(); ++i)
  {
    dqd_[i].angularVelocity = 0.0 * radian_per_second;
  }

  switch(ch)
  {
  case '1':
    dqd_[0].angularVelocity = qd_ * radian_per_second;
    break;
  case 'q':
    dqd_[0].angularVelocity = -qd_ * radian_per_second;
    break;
  case '2':
    dqd_[1].angularVelocity = qd_ * radian_per_second;
    break;
  case 'w':
    dqd_[1].angularVelocity = -qd_ * radian_per_second;
    break;
  case '3':
    dqd_[2].angularVelocity = qd_ * radian_per_second;
    break;
  case 'e':
    dqd_[2].angularVelocity = -qd_ * radian_per_second;
    break;
  case '4':
    dqd_[3].angularVelocity = qd_ * radian_per_second;
    break;
  case 'r':
    dqd_[3].angularVelocity = -qd_ * radian_per_second;
    break;
  case '5':
    dqd_[4].angularVelocity = qd_ * radian_per_second;
    break;
  case 't':
    dqd_[4].angularVelocity = -qd_ * radian_per_second;
    break;
  default:
    break;
  };

  manipulator_->setJointData(dqd_);
}

void TeleOperation::stop()
{
  this->stopBase();
  this->stopManipulator();
  manipulator_->getArmGripper().close();
}

void TeleOperation::stopBase()
{
  dvx_ = 0 * meter_per_second;
  dvy_ = 0 * meter_per_second;
  dvr_ = 0 * radian_per_second;

  base_->setBaseVelocity(dvx_, dvy_, dvr_);
}

void TeleOperation::stopManipulator()
{
  for(unsigned int i = 0; i < dqd_.size(); ++i)
  {
    dqd_[i].angularVelocity = 0.0 * radian_per_second;
  }

  manipulator_->setJointData(dqd_);
}
