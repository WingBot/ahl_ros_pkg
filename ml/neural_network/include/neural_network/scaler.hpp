/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
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

#ifndef __NEURAL_NETWORK_SCALER_HPP
#define __NEURAL_NETWORK_SCALER_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "neural_network/training_data.hpp"

namespace nn
{

  class Scaler
  {
  public:
    Scaler(double max = 0.9, double min = 0.1);

    void init(const std::string& yaml);
    void init(const TrainingDataPtr& data);
    void init(std::vector<double>& max_in, std::vector<double>& min_in,
              std::vector<double>& max_out, std::vector<double>& min_out);
    void init(Eigen::MatrixXd& max_in, Eigen::MatrixXd& min_in,
              Eigen::MatrixXd& max_out, Eigen::MatrixXd& min_out);

    void normalize(TrainingDataPtr& data);
    void normalizeInput(std::vector<double>& input);
    void normalizeInput(Eigen::MatrixXd& input);
    void denormalizeOutput(std::vector<double>& output);
    void denormalizeOutput(Eigen::MatrixXd& output);

    void save(const std::string& yaml);
    void print();

  private:
    void calcMaxMin(const std::vector<Eigen::MatrixXd>& src, Eigen::MatrixXd& max, Eigen::MatrixXd& min);
    void normalize(Eigen::MatrixXd& src, const Eigen::MatrixXd& min, const Eigen::MatrixXd& range);
    void denormalize(Eigen::MatrixXd& src, const Eigen::MatrixXd& min, const Eigen::MatrixXd& range);

    double max_; // all values will be smaller than max_
    double min_; // all values will be bigger than min_
    double range_;

    Eigen::MatrixXd max_in_;
    Eigen::MatrixXd min_in_;
    Eigen::MatrixXd range_in_;

    Eigen::MatrixXd max_out_;
    Eigen::MatrixXd min_out_;
    Eigen::MatrixXd range_out_;
  };

  typedef boost::shared_ptr<Scaler> ScalerPtr;
}

#endif /* __NEURAL_NETWORK_SCALER_HPP */
