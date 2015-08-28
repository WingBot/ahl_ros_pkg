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

#ifndef __NEURAL_NETWORK_CORE_BACK_PROPAGATION_HPP
#define __NEURAL_NETWORK_CORE_BACK_PROPAGATION_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "neural_network/config.hpp"
#include "neural_network/training_data.hpp"
#include "neural_network/core/layer.hpp"
#include "neural_network/core/forward_calculator.hpp"
#include "neural_network/core/backward_calculator.hpp"
#include "neural_network/core/activation.hpp"

namespace nn
{
  class BackPropagation
  {
  public:
    BackPropagation(const ConfigPtr& config);
    void train(const TrainingDataPtr& data);

    double getCost(const TrainingDataPtr& data);

    void initDw();
    void correctWeight(unsigned int idx, const Eigen::MatrixXd& dw);
    void copyLayerTo(std::vector<LayerPtr>& layer)
    {
      layer = layer_;
    }

    const unsigned int getDataSize() const
    {
      return data_size_;
    }

    const std::vector<Eigen::MatrixXd>& getDw() const
    {
      return dw_;
    }

  private:
    void applyGradientDescent();

    double learning_rate_;
    double momentum_rate_;

    unsigned int data_size_;

    std::vector<LayerPtr> layer_;
    std::vector<Eigen::MatrixXd> bp_neuron_;
    std::vector<Eigen::MatrixXd> dw_;
    std::vector<Eigen::MatrixXd> pre_dw_;
    std::vector<Eigen::MatrixXd> dw_zero_;

    ForwardCalculatorPtr forward_calculator_;
    BackwardCalculatorPtr backward_calculator_;
  };

  typedef boost::shared_ptr<BackPropagation> BackPropagationPtr;
}

#endif /* __NEURAL_NETWORK_CORE_BACK_PROPAGATION_HPP */
