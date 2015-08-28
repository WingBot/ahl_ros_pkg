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

#ifndef __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP

#include <yaml-cpp/yaml.h>
#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  typedef boost::shared_ptr<youbot::YouBotBase> YouBotBasePtr;
  typedef boost::shared_ptr<youbot::YouBotManipulator> YouBotManipulatorPtr;

  namespace youbot_if
  {
    static const std::string CONFIG_BASE = "youbot-base";
    static const std::string CONFIG_MANIPULATOR = "youbot-manipulator";

    namespace tag
    {
      static const std::string JOINT = "joint";
      static const std::string OFFSET = "offset";
      static const std::string FLIP_DIRECTION = "flip_direction";
      static const std::string P_CURRENT = "Kp_current";
      static const std::string I_CURRENT = "Ki_current";
      static const std::string D_CURRENT = "Kd_current";
    }
  }

  class YoubotInterface : public Interface
  {
  public:
    YoubotInterface(unsigned int dof, const std::string& ahl_cfg_path, const std::string& cfg_path, bool do_calibration);
    ~YoubotInterface();

    bool getJointStates(Eigen::VectorXd& q); 
    bool getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq);
    bool getJointEfforts(Eigen::VectorXd& tau);
    bool getOdometry(Eigen::Vector3d& q);
    bool getOdometry(Eigen::Vector3d& q, Eigen::Vector3d& dq);
    bool applyJointEfforts(const Eigen::VectorXd& tau);
    bool applyBaseVelocity(const Eigen::Vector3d& v);

  private:
    void init(const std::string& ahl_cfg_path);
    void checkTag(const YAML::Node& node, const std::string& tag);

    unsigned int dof_;
    unsigned int base_dof_;
    unsigned int arm_dof_;
    YouBotBasePtr base_;
    YouBotManipulatorPtr mnp_;

    Eigen::VectorXd q_offset_;
    Eigen::VectorXi flip_direction_;
  };

}

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP */
