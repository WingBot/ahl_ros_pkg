/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

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
