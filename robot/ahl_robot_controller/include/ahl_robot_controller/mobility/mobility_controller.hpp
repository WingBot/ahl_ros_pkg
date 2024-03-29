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

#ifndef __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ahl_robot/robot/mobility.hpp>
#include "ahl_robot_controller/exception.hpp"

namespace ahl_ctrl
{

  class MobilityController
  {
  public:
    virtual ~MobilityController() {}

    virtual void computeBaseVelocityFromTorque(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& tau, Eigen::VectorXd& v_base)
    {
      throw ahl_ctrl::Exception("MobilityController::computeBaseVelocity", "This virtual function is not implemented.");
    }

    virtual void computeWheelVelocityFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& v_wheel)
    {
      throw ahl_ctrl::Exception("MobilityController::computeWheelVelocity", "This virtual function is not implemented.");
    }

    virtual void computeWheelTorqueFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& tau_wheel)
    {
      throw ahl_ctrl::Exception("MobilityController::computeWheelTorque", "This virtual function is not implemented.");
    }
  };

  typedef boost::shared_ptr<MobilityController> MobilityControllerPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP */
