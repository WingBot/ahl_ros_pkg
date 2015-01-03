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

#ifndef __NEURAL_NETWORK_CORE_NEURAL_NETWORK_HPP
#define __NEURAL_NETWORK_CORE_NEURAL_NETWORK_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "neural_network/config.hpp"
#include "neural_network/training_data.hpp"
#include "neural_network/scaler.hpp"

#include "neural_network/core/multi_threaded_back_propagation.hpp"
#include "neural_network/core/layer.hpp"
#include "neural_network/core/forward_calculator.hpp"

namespace nn
{

  class NeuralNetwork
  {
  public:
    NeuralNetwork();

    void init(const ConfigPtr& config);
    void train(const TrainingDataPtr& data, const std::string& yaml);

    const std::vector<LayerPtr>& getLayer() const
    {
      return layer_;
    }

    Eigen::MatrixXd getOutput(Eigen::MatrixXd& input);

  private:
    void save(const std::string& yaml);
    void loadWeight();

    bool initialized_;
    ConfigPtr config_;
    MultiThreadedBackPropagationPtr multi_threaded_back_propagation_;

    std::vector<LayerPtr> layer_;
    ForwardCalculatorPtr forward_calculator_;
  };

  typedef boost::shared_ptr<NeuralNetwork> NeuralNetworkPtr;
}

#endif /* __NEURAL_NETWORK_CORE_NEURAL_NETWORK_HPP */
