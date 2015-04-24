#ifndef __AHL_YOUBOT_SAMPLE_TELE_OPERATION_HPP
#define __AHL_YOUBOT_SAMPLE_TELE_OPERATION_HPP

#include <vector>
#include <boost/shared_ptr.hpp>

#include <curses.h>

#include <ros/ros.h>
#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

namespace ahl_youbot
{

  class TeleOperation
  {
  public:
    TeleOperation();
    ~TeleOperation();
    void run();

  private:
    typedef boost::shared_ptr<youbot::YouBotBase> YouBotBasePtr;
    typedef boost::shared_ptr<youbot::YouBotManipulator> YouBotManipulatorPtr;

    void init();
    void printHotKeys();
    void getCommand();
    void getBaseCommand(int ch);
    void getManipulatorCommand(int ch);
    void setBaseCommandToZero();
    void setManipulatorCommandToZero();

    bool initialized_;
    bool running_;

    double duration_;

    double scalar_vx_; // longitudinal velocity
    double scalar_vy_; // transversal velocity
    double scalar_vr_; // transversal velocity

    YouBotBasePtr base_;
    YouBotManipulatorPtr manipulator_;

    youbot::JointVelocitySetpoint setpoint_;

    quantity<si::velocity> vx_; // longitudinal velocity
    quantity<si::velocity> vy_; // transversal velocity
    quantity<si::angular_velocity> vr_; // angular velocity
  };

}

#endif /* __AHL_YOUBOT_SAMPLE_TELE_OPERATION_HPP */
