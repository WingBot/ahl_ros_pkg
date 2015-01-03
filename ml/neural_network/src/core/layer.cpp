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

#include <ctime>

#include "neural_network/exceptions.hpp"
#include "neural_network/core/layer.hpp"

using namespace nn;

Layer::Layer(unsigned int neuron_num)
{
  neuron_ = Eigen::MatrixXd::Zero(neuron_num + 1, 1);
  neuron_.coeffRef(neuron_num, 0) = 1.0;
}

void Layer::setNext(const LayerPtr& next)
{
  if(next == NULL)
  {
    throw nn::Exception("nn::Layer::setNext", "Next layer pointer is null.");
  }

  next_ = next;

  std::srand(static_cast<unsigned int>(std::time(NULL)));
  w_ = Eigen::MatrixXd::Random(next_->getNeuronSize() - 1, neuron_.rows());

  // avoid neuron's value to get large drastically
  const double coeff_w = 0.01;
  w_ *= coeff_w;
}

void Layer::setPre(const LayerPtr& pre)
{
  if(pre == NULL)
  {
    throw nn::Exception("nn::Layer::setPre", "Previous layer pointer is null.");
  }

  pre_ = pre;
}
