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

#ifndef __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP
#define __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP

#include <map>

#include "ahl_youbot_server/ahl_robot_actions.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/action/action_client_base.hpp"

namespace ahl_youbot
{

  template<typename ActionType, typename GoalType>
  class ActionClient : public ActionClientBase
  {
  public:
    ActionClient(const std::string& action_name)
    {
      client_ = boost::shared_ptr< actionlib::SimpleActionClient<ActionType> >(
        new actionlib::SimpleActionClient<ActionType>(action_name, true));
    }

    void cancelAllGoals()
    {
      client_->cancelAllGoals();
    }

    void cancelGoal()
    {
      client_->cancelGoal();
    }

    void cancelGoalsAtAndBeforeTime(const ros::Time& time)
    {
      client_->cancelGoalsAtAndBeforeTime(time);
    }

    actionlib::SimpleClientGoalState getState()
    {
      return client_->getState();
    }

    bool isServerConnected()
    {
      return client_->isServerConnected();
    }

    void sendGoal(void* goal)
    {
      client_->sendGoal(*(static_cast<GoalType*>(goal)));
    }

    void stopTrackingGoal()
    {

    }

    bool waitForResult(const ros::Duration& timeout = ros::Duration(0, 0))
    {
      return client_->waitForResult(timeout);
    }

    bool waitForServer(const ros::Duration& timeout = ros::Duration(0, 0))
    {
      return client_->waitForServer(timeout);
    }

  private:
    boost::shared_ptr< actionlib::SimpleActionClient<ActionType> > client_;
  };

  template<class ActionType,class GoalType>
  struct ActionClientPtr
  {
    typedef boost::shared_ptr< ActionClientPtr<ActionType, GoalType> > client;
  };
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP */
