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

    void control();
    void controlBase(int ch);
    void controlGripper(int ch);
    void controlManipulator(int ch);

    void stop();
    void stopBase();
    void stopManipulator();

    bool initialized_;
    bool running_;

    double duration_;

    double vx_; // longitudinal velocity
    double vy_; // transversal velocity
    double vr_; // angular velocity (yaw)
    double qd_; // angular velocity of manipulator

    quantity<velocity> dvx_; // desired longitudinal velocity
    quantity<velocity> dvy_; // desired transversal velocity
    quantity<angular_velocity> dvr_; // desired angular velocity

    YouBotBasePtr base_;
    YouBotManipulatorPtr manipulator_;

    std::vector<youbot::JointVelocitySetpoint> dqd_; // desired angular velocity of manipulator
  };

}

#endif /* __AHL_YOUBOT_SAMPLE_TELE_OPERATION_HPP */
