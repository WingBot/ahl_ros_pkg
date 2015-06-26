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

#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ahl_robot_controller/robot_controller.hpp>

#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer(
      const std::string& robot_name, const std::string& mnp_name, const std::string& yaml,
      double period, double servo_period, bool use_real_robot);

    void start();
    void stop();
    void changeActionTo(Action::Type action_type)
    {
      action_type_ = action_type;
    }

    const std::string& getActionName(Action::Type type)
    {
      return action_[type]->getActionName();
    }

  private:
    void timerCB(const ros::TimerEvent&);
    void servoTimerCB(const ros::TimerEvent&);

    bool updated_model_;

    std::map<Action::Type, ActionPtr> action_;
    Action::Type action_type_;

    InterfacePtr interface_;
    ahl_robot::RobotPtr robot_;
    ahl_robot::TfPublisherPtr tf_pub_;
    std::string mnp_name_;
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd q_arm_;
    Eigen::VectorXd dq_arm_;
    Eigen::VectorXd q_base_;
    Eigen::VectorXd dq_base_;

    ahl_ctrl::RobotControllerPtr controller_;

    boost::mutex mutex_;
    ros::Timer timer_;
    ros::Timer servo_timer_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
