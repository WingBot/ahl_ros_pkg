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

#include <std_utils/io_utils.hpp>
#include <std_utils/yaml_utils.hpp>

#include "neural_network/core/neural_network.hpp"
#include "neural_network/core/activation.hpp"
#include "neural_network/core/sigmoid.hpp"
#include "neural_network/core/sigmoid_table.hpp"
#include "neural_network/core/piecewise_linear.hpp"
#include "neural_network/core/rectified_linear.hpp"
#include "neural_network/exceptions.hpp"

using namespace nn;

NeuralNetwork::NeuralNetwork()
  : initialized_(false)
{
}

void NeuralNetwork::init(const ConfigPtr& config)
{
  config_ = config;
  initialized_ = true;

  for(unsigned int i = 0; i < config_->getNeuronNum().size(); ++i)
  {
    layer_.push_back(LayerPtr(new Layer(config_->getNeuronNum()[i])));
  }

  for(unsigned int i = 0; i < config_->getNeuronNum().size(); ++i)
  {
    if(i > 0)
    {
      layer_[i]->setPre(layer_[i - 1]);
    }

    if(i < config_->getNeuronNum().size() - 1)
    {
      layer_[i]->setNext(layer_[i + 1]);
    }
  }

  if(config_->enableBackPropagation())
  {
    multi_threaded_back_propagation_ = MultiThreadedBackPropagationPtr(new MultiThreadedBackPropagation(config));
  }
  else
  {
    this->loadWeight();
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
}

void NeuralNetwork::train(const TrainingDataPtr& data, const std::string& yaml)
{
  if(initialized_ == false)
  {
    std::stringstream msg;
    msg << "NeuralNetwork is not initialized." << std::endl
        << "        Please call NeuralNetwork::init(const ConfigPtr& config).";

    throw nn::Exception("NeuralNetwork::train", msg.str());
  }

  if(multi_threaded_back_propagation_ == NULL)
  {
    std::stringstream msg;
    msg << "Cannot call NeuralNetwork::train(const TrainingDataPtr& data)." << std::endl
        << "        multi_threaded_back_propagation_->get() is NULL." << std::endl
        << "        Please check enable_back_propagation value in your config file \"***.yaml\"";

    throw nn::Exception("NeuralNetwork::train", msg.str());
  }

  multi_threaded_back_propagation_->train(data);
  multi_threaded_back_propagation_->copyLayerTo(layer_);

  this->save(yaml);
}

Eigen::MatrixXd NeuralNetwork::getOutput(Eigen::MatrixXd& input)
{
  forward_calculator_->calculate(input, layer_);
  return layer_[layer_.size() - 1]->getNeuron().block(0, 0, layer_[layer_.size() - 1]->getNeuron().rows() - 1, 1);
}

void NeuralNetwork::save(const std::string& yaml)
{
  std::ofstream ofs(yaml.c_str());

  if(ofs.fail())
  {
    std::stringstream msg;
    msg << "Could not open \"" << yaml << "\".";
    throw nn::Exception("NeuralNetwork::save", msg.str());
  }

  ofs << "this file is a configuration file for neural_network package." << std::endl;

  ofs << std::endl;

  ofs << "enable_back_propagation : ";
  if(config_->enableBackPropagation())
    ofs << "true" << std::endl;
  else
    ofs << "false" << std::endl;

  ofs << "thread_num : " << config_->getThreadNum() << std::endl
      << "batch_size : " << config_->getBatchSize() << std::endl
      << "learning_rate : " << config_->getLearningRate() << std::endl
      << "momentum_rate : " << config_->getMomentumRate() << std::endl
      << "activation_gain : " << config_->getActivationGain() << std::endl
      << "reference_cost : " << config_->getReferenceCost() << std::endl
      << "max_iterations : " << config_->getMaxIterations() << std::endl
      << "calc_cost_interval : " << config_->getCalcCostInterval() << std::endl
      << "activation_type : " << config_->getActivationType() << std::endl;

  ofs << std::endl;

  ofs << "layer :" << std::endl;
  for(unsigned int i = 0; i < layer_.size(); ++i)
  {
    ofs << "- " << layer_[i]->getNeuron().rows() - 1 << std::endl;
  }

  ofs << std::endl;

  for(unsigned int l = 0; l < layer_.size() - 1; ++l)
  {
    std::stringstream name;
    name << "weight" << l;

    ofs << name.str() << " : " << std::endl;
    for(unsigned int i = 0; i < layer_[l]->getW().rows(); ++i)
    {
      ofs << "- [";
      for(unsigned int j = 0; j < layer_[l]->getW().cols() - 1; ++j)
      {
        ofs << layer_[l]->getW().coeff(i, j) << ", ";
      }
      if(layer_[l]->getW().cols() - 1 >= 0)
        ofs << layer_[l]->getW().coeff(i, layer_[l]->getW().cols() - 1);
      
      ofs << "]" << std::endl;
    }
    ofs << std::endl;
  }
}

void NeuralNetwork::loadWeight()
{

}
