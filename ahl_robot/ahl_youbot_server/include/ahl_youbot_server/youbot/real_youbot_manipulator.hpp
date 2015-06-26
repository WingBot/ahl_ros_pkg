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

#ifndef __AHL_YOUBOT_SERVER_REAL_YOUBOT_MANIPULATOR_HPP
#define __AHL_YOUBOT_SERVER_REAL_YOUBOT_MANIPULATOR_HPP

#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include "ahl_youbot_server/youbot/youbot_manipulator.hpp"

namespace ahl_youbot
{

  class RealYouBotManipulator : public YouBotManipulator
  {
  public:
    RealYouBotManipulator();

    // arm
    virtual void doJointCommutation();
    virtual void calibrateManipulator(const bool force_calibration = false);
    virtual int getNumberJoints();

    virtual void setAngle(const std::vector<double>& q);
    virtual void setVelocity(const std::vector<double>& dq);
    virtual void setTorque(const std::vector<double>& tau);

    virtual void getAngle(std::vector<double>& q);
    virtual void getVelocity(std::vector<double>& dq);
    virtual void setTorque(std::vector<double>& tau);

    // gripper
    virtual void calibrateGripper(const bool force_calibration = false);
    virtual void open();
    virtual void close();
    virtual void setSpace(double space);
    virtual void getSpace(double& space);

  private:
    typedef boost::shared_ptr<youbot::YouBotManipulator> YouBotManipulatorPtr;

    YouBotManipulatorPtr manipulator_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_REAL_YOUBOT_MANIPULATOR_HPP */
