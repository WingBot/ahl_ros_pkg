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

#ifndef __AHL_YOUBOT_SERVER_ACTION_HPP
#define __AHL_YOUBOT_SERVER_ACTION_HPP

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ahl_robot/ahl_robot.hpp>

namespace ahl_youbot
{

  class Action
  {
  public:
    enum Type
    {
      FLOAT,
      SET_JOINT,
      JOINT_SPACE_CONTROL,
      TASK_SPACE_CONTROL,
      TASK_SPACE_HYBRID_CONTROL,
      ACTION_NUM,
    };

    Action(const std::string& action_name)
      : action_name_(action_name) {}
    virtual ~Action() {}

    virtual void execute(void* goal) = 0;

    virtual const std::string& getActionName() const
    {
      return action_name_;
    }

  protected:
    const ros::NodeHandle& getNodeHandle() const
    {
      return nh_;
    }

  private:
    ros::NodeHandle nh_;
    std::string action_name_;
  };

  typedef boost::shared_ptr<Action> ActionPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_HPP */
