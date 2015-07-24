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

#ifndef __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP

#include <map>
#include <list>
#include <boost/shared_ptr.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/param.hpp"
#include "ahl_robot_controller/mobility/mobility_controller.hpp"
#include "ahl_robot_controller/task/task.hpp"
#include "ahl_robot_controller/task/multi_task.hpp"

namespace ahl_ctrl
{

  class RobotController
  {
  public:
    RobotController();

    void init(const ahl_robot::RobotPtr& robot, const std::string& mnp_name);
    void init(const ahl_robot::RobotPtr& robot, const std::vector<std::string>& mnp_name);
    void addTask(const TaskPtr& task, int priority);
    void clearTask();
    void updateModel();
    void computeGeneralizedForce(Eigen::VectorXd& tau);
    void computeBaseVelocityFromTorque(
      const Eigen::VectorXd& tau, Eigen::VectorXd& v_base, int mobility_dof = 3);
    void computeWheelVelocityFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& v_wheel);
    void computeWheelTorqueFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& tau_wheel);

  private:
    ParamPtr param_;
    MultiTaskPtr multi_task_;
    ahl_robot::RobotPtr robot_;
    std::vector<ahl_robot::ManipulatorPtr> mnp_;
    MobilityControllerPtr mobility_controller_;
    ahl_robot::MobilityPtr mobility_;
    unsigned int dof_;
  };

  typedef boost::shared_ptr<RobotController> RobotControllerPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP */
