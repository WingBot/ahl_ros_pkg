#include "ahl_youbot_sample/tele_operation/tele_operation.hpp"

using namespace ahl_youbot;

TeleOperation::TeleOperation()
  : initialized_(false), running_(false)
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("ahl_youbot_sample");

  std::string cfg_youbot_base;
  std::string cfg_youbot_manipulator;

  local_nh.param<double>("tele_operation/duration", duration_, 200);
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_base", cfg_youbot_base, "youbot-base");
  local_nh.param<std::string>(
    "tele_operation/cfg/youbot_manipulator", cfg_youbot_manipulator, "youbot-manipulator");
  local_nh.param<double>("tele_operation/scalar_vx", scalar_vx_, 0.05);
  local_nh.param<double>("tele_operation/scalar_vy", scalar_vy_, 0.05);
  local_nh.param<double>("tele_operation/scalar_vr", scalar_vr_, 0.2);

  base_ = YouBotBasePtr(
    new youbot::YouBotBase(cfg_youbot_base, YOUBOT_CONFIGURATIONS_DIR));
  base_->doJointCommutation();

  manipulator_ = YouBotManipulatorPtr(
    new youbot::YouBotManipulator(cfg_youbot_manipulator, YOUBOT_CONFIGURATIONS_DIR));
  manipulator_->doJointCommutation();
  manipulator_->calibrateManipulator();
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
    this->setBaseCommandToZero();
    this->setManipulatorCommandToZero();
    this->getCommand();

    std::cout << "(vx, vy, vr) = ("
              << vx_ << ", " << vy_ << ", " << vr_ << ")\r" << std::endl;
    //base_->setBaseVelocity(vx_, vy_, vr_);
    refresh();

    ros::Duration(duration_).sleep();
  }
}

void TeleOperation::init()
{
  static_cast<void>(initscr()); // initialize the curses library
  keypad(stdscr, TRUE); // enable keyboard mapping
  static_cast<void>(nonl()); // tell curses not to do NL->CR/NL on output
  static_cast<void>(cbreak()); // take input chars one at a time, no wait for \n

  def_prog_mode();
  refresh();

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

void TeleOperation::getCommand()
{
  int ch = getch();

  if(ch == KEY_UP || ch == KEY_DOWN || ch == KEY_LEFT || ch == KEY_RIGHT ||
     ch == 'z' || ch == 'x')
  {
    this->getBaseCommand(ch);
  }
  else
  {
    this->getManipulatorCommand(ch);
  }
}

void TeleOperation::getBaseCommand(int ch)
{
  switch(ch)
  {
  case KEY_UP:
    vx_ = scalar_vx_ * meter_per_second;
    break;
  case KEY_DOWN:
    vx_ = -scalar_vx_ * meter_per_second;
    break;
  case KEY_LEFT:
    vy_ = scalar_vy_ * meter_per_second;
    break;
  case KEY_RIGHT:
    vy_ = -scalar_vy_ * meter_per_second;
    break;
  case 'z':
    vr_ = scalar_vr_ * radian_per_second;
    break;
  case 'x':
    vr_ = -scalar_vr_ * radian_per_second;
    break;
  default:
    break;
  };
}

void TeleOperation::getManipulatorCommand(int ch)
{
  switch(ch)
  {
  default:
    break;
  };
}

void TeleOperation::setBaseCommandToZero()
{
  vx_ = 0 * meter_per_second;
  vy_ = 0 * meter_per_second;
  vr_ = 0 * radian_per_second;
}

void TeleOperation::setManipulatorCommandToZero()
{

}
