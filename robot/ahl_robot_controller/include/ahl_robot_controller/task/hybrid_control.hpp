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

#ifndef __AHL_ROBOT_CONTROLLER_HYBRID_CONTROL_HPP
#define __AHL_ROBOT_CONTROLLER_HYBRID_CONTROL_HPP

#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/task/task.hpp"
#include "ahl_robot_controller/task/position_control.hpp"
#include "ahl_robot_controller/task/orientation_control.hpp"

namespace ahl_ctrl
{

  class HybridControl : public Task
  {  
  public:
    enum PreDefinedTaskList
    {
      PositionControl,
      OrientationControl,
      PreDefinedTaskNum,
    };

    HybridControl(const ahl_robot::ManipulatorPtr& mnp, const std::string& target_link, const Eigen::Matrix3d& Rf, const Eigen::Matrix3d& Rm, double zero_thresh = 1e-4, double eigen_thresh = 1e-3);
    virtual void setGoal(const Eigen::MatrixXd& ref); // 12 dimension, xyz, rpy, fxfyfz, mxmymz
    virtual void updateModel();
    virtual void computeGeneralizedForce(Eigen::VectorXd& tau);
    virtual bool haveNullSpace() { return true; }

  private:
    bool updated_;

    std::map<HybridControl::PreDefinedTaskList, TaskPtr> tasks_;

    unsigned int idx_;

    Eigen::Matrix3d I3_;

    Eigen::VectorXd fd_; // Desired forces and moments (Fx, Fy, Fz, Mx, My, Mz)^T
    Eigen::Matrix3d Rf_; // Orientation of the frame for force control w.r.t base
    Eigen::Matrix3d Rm_; // Orientation of the frame for moment control w.r.t base

    double zero_thresh_;

    Eigen::Matrix3d sigma_f_;
    Eigen::Matrix3d sigma_m_;
    Eigen::Matrix3d sigma_f_bar_;
    Eigen::Matrix3d sigma_m_bar_;
    Eigen::MatrixXd omega_;
    Eigen::MatrixXd omega_bar_;

    Eigen::Matrix3d Jv_;
    Eigen::Matrix3d lambda_v_;
    Eigen::Matrix3d lambda_v_inv_;
    Eigen::MatrixXd Jv_dyn_inv_;
    Eigen::Matrix3d Jw_;
    Eigen::Matrix3d lambda_w_;
    Eigen::Matrix3d lambda_w_inv_;
    Eigen::MatrixXd Jw_dyn_inv_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_HYBRID_CONTROL_HPP */
