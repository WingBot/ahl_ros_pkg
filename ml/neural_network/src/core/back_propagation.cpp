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

#include "neural_network/core/back_propagation.hpp"
#include "neural_network/core/activation.hpp"
#include "neural_network/core/sigmoid.hpp"
#include "neural_network/core/sigmoid_table.hpp"
#include "neural_network/core/piecewise_linear.hpp"
#include "neural_network/core/rectified_linear.hpp"

#include "neural_network/exceptions.hpp"

using namespace nn;

BackPropagation::BackPropagation(const ConfigPtr& config)
  : learning_rate_(config->getLearningRate()),
    momentum_rate_(config->getMomentumRate()),
    data_size_(0)
{
  for(unsigned int i = 0; i < config->getNeuronNum().size(); ++i)
  {
    layer_.push_back(LayerPtr(new Layer(config->getNeuronNum()[i])));
  }

  bp_neuron_.resize(config->getNeuronNum().size());
  dw_.resize(config->getNeuronNum().size());
  pre_dw_.resize(config->getNeuronNum().size());
  dw_zero_.resize(config->getNeuronNum().size());

  for(unsigned int i = 0; i < config->getNeuronNum().size(); ++i)
  {
    if(i > 0)
    {
      layer_[i]->setPre(layer_[i - 1]);
      bp_neuron_[i] = Eigen::MatrixXd::Zero(layer_[i]->getNeuronSize() - 1, 1);
    }

    if(i < config->getNeuronNum().size() - 1)
    {
      layer_[i]->setNext(layer_[i + 1]);
      dw_[i]      = Eigen::MatrixXd::Zero(layer_[i + 1]->getNeuronSize() - 1, layer_[i]->getNeuronSize());
      pre_dw_[i]  = Eigen::MatrixXd::Zero(layer_[i + 1]->getNeuronSize() - 1, layer_[i]->getNeuronSize());
      dw_zero_[i] = Eigen::MatrixXd::Zero(layer_[i + 1]->getNeuronSize() - 1, layer_[i]->getNeuronSize());
    }
  }

  ActivationPtr activation;
  if(config->getActivationType() == "sigmoid")
  {
    activation = ActivationPtr(new Sigmoid(config->getActivationGain()));
  }
  else if(config->getActivationType() == "sigmoid_table")
  {
    activation = ActivationPtr(new SigmoidTable(config->getActivationGain()));
  }
  else if(config->getActivationType() == "piecewise_linear")
  {
    activation = ActivationPtr(new PiecewiseLinear(config->getActivationGain()));
  }
  else if(config->getActivationType() == "rectified_linear")
  {
    activation = ActivationPtr(new RectifiedLinear(config->getActivationGain()));
  }

  forward_calculator_ = ForwardCalculatorPtr(new ForwardCalculator(activation));
  backward_calculator_ = BackwardCalculatorPtr(new BackwardCalculator(activation));
}

void BackPropagation::train(const TrainingDataPtr& data)
{
  data_size_ = data->getSize();

  for(unsigned int i = 0; i < data_size_; ++i)
  {
    forward_calculator_->calculate(data->getInput()[i], layer_);
    backward_calculator_->calculate(data->getOutput()[i], layer_, bp_neuron_);
    this->applyGradientDescent();
  }
}

double BackPropagation::getCost(const TrainingDataPtr& data)
{
  double cost = 0.0;

  for(unsigned int i = 0; i < data->getSize(); ++i)
  {
    forward_calculator_->calculate(data->getInput()[i], layer_);
    unsigned int last = layer_.size() - 1;
    unsigned int rows = layer_[last]->getNeuron().rows();
    unsigned int cols = layer_[last]->getNeuron().cols();
    cost += (layer_[last]->getNeuron().block(0, 0, rows - 1, cols) - data->getOutput()[i]).norm();
  }

  //return 0.5 * cost;
  return 0.5 * cost / data->getSize();
}

void BackPropagation::initDw()
{
  for(unsigned int i = 0; i < layer_.size() - 1; ++i)
  {
    dw_[i] = dw_zero_[i];
  }
}

void BackPropagation::correctWeight(unsigned int idx, const Eigen::MatrixXd& dw)
{
  layer_[idx]->getWRef() += dw;
}

void BackPropagation::applyGradientDescent()
{
  for(unsigned int i = 0; i < layer_.size() - 1; ++i)
  {
    dw_[i] += -learning_rate_ * bp_neuron_[i + 1] * layer_[i]->getNeuron().transpose();
  }
}
