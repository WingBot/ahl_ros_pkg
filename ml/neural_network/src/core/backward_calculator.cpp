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

#include "neural_network/core/backward_calculator.hpp"
#include "neural_network/exceptions.hpp"

using namespace nn;

BackwardCalculator::BackwardCalculator(const ActivationPtr& activation)
  : activation_(activation)
{
}

void BackwardCalculator::calculate(const Eigen::MatrixXd& output, std::vector<LayerPtr>& layer,
                                   std::vector<Eigen::MatrixXd>& bp_neuron)
{
  unsigned int layer_size_minus = layer.size() - 1;

  for(unsigned int i = 0; i < output.rows(); ++i)
  {
    double n = layer[layer_size_minus]->getNeuron().coeff(i, 0);
    bp_neuron[layer_size_minus].coeffRef(i, 0) = activation_->getDerivative(n) * (n - output.coeff(i, 0));
  }

  for(unsigned int i = layer_size_minus - 1; i > 0; --i)
  {
    bp_neuron[i] = layer[i]->getW().transpose().block(0, 0, layer[i]->getW().cols() - 1, layer[i]->getW().rows())
                 * bp_neuron[i + 1];

    for(unsigned int j = 0; j < bp_neuron[i].rows(); ++j)
    {
      bp_neuron[i].coeffRef(j, 0) = activation_->getDerivative(layer[i]->getNeuron().coeff(j, 0))
                                  * bp_neuron[i].coeff(j, 0);
    }
  }
}

