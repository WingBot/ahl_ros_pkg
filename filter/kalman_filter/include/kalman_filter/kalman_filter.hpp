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

#ifndef __KALMAN_FILTER_KALMAN_FILTER_HPP
#define __KALMAN_FILTER_KALMAN_FILTER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "kalman_filter/normal_distribution.hpp"

namespace kf
{

  class KalmanFilter
  {
  public:
    void setRandomVariables(const NormalDistributionPtr& state,
                            const NormalDistributionPtr& uncertainty,
                            const NormalDistributionPtr& msr_noise);
    void setLinearModel(const Eigen::MatrixXd& coeff_of_mean,
                        const Eigen::MatrixXd& coeff_of_ctrl_data,
                        const Eigen::MatrixXd& coeff_of_msr_data);
    void estimate(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data, NormalDistributionPtr& state);

    const NormalDistributionPtr& getState() const
    {
      return state_;
    }

    const NormalDistributionPtr& getPredictedState() const
    {
      return predicted_state_;
    }

  private:
    void checkMatrixSize(const Eigen::MatrixXd& ctrl_data, const Eigen::MatrixXd& msr_data);

    NormalDistributionPtr state_;
    NormalDistributionPtr predicted_state_;
    NormalDistributionPtr uncertainty_;
    NormalDistributionPtr msr_noise_;

    Eigen::MatrixXd A_;
    Eigen::MatrixXd B_;
    Eigen::MatrixXd C_;
  };

  typedef boost::shared_ptr<KalmanFilter> KalmanFilterPtr;
}

#endif /* __KALMAN_FILTER_KALMAN_FILTER_HPP */
