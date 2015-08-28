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

#include <iostream>
#include "neural_network/core/forward_calculator.hpp"
#include "neural_network/exceptions.hpp"

using namespace nn;

ForwardCalculator::ForwardCalculator(const ActivationPtr& activation)
  : activation_(activation)
{
}

void ForwardCalculator::calculate(const Eigen::MatrixXd& input, std::vector<LayerPtr>& layer)
{
  layer[0]->getNeuronRef().block(0, 0, layer[0]->getNeuronRef().rows() - 1, 1) = input;

  for(unsigned int i = 1; i < layer.size(); ++i)
  {
    unsigned int neuron_rows_minus = layer[i]->getNeuronSize() - 1;
    layer[i]->getNeuronRef().block(0, 0, neuron_rows_minus, 1)
      = layer[i]->getPre()->getW() * layer[i]->getPre()->getNeuron();

    for(unsigned int j = 0; j < neuron_rows_minus; ++j)
    {
      layer[i]->getNeuronRef().coeffRef(j, 0) = activation_->getOutput(layer[i]->getNeuronRef().coeff(j, 0));
    }
  }
}
