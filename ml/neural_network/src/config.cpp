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

#include <fstream>
#include "std_utils/yaml_loader.hpp"
#include "neural_network/config.hpp"
#include "neural_network/exceptions.hpp"
#include "neural_network/warning.hpp"

using namespace nn;

Config::Config()
  : enable_back_propagation_(true),
    thread_num_(1),
    batch_size_(1),
    learning_rate_(0.01),
    momentum_rate_(0.90),
    activation_gain_(1.0),
    reference_cost_(0.001),
    max_iterations_(1000),
    calc_cost_interval_(100),
    activation_type_(std::string("sigmoid")),
    neuron_num_(0)
{
}

void Config::init(const std::string& yaml_name)
{
  const std::string src = "nn::Config::init";

  try
  {
    std_utils::YAMLLoader yaml_loader(yaml_name);

    yaml_loader.loadValue("enable_back_propagation", enable_back_propagation_);
    yaml_loader.loadValue("thread_num", thread_num_);
    yaml_loader.loadValue("batch_size", batch_size_);
    yaml_loader.loadValue("learning_rate", learning_rate_);
    yaml_loader.loadValue("momentum_rate", momentum_rate_);
    yaml_loader.loadValue("activation_gain", activation_gain_);
    yaml_loader.loadValue("reference_cost", reference_cost_);
    yaml_loader.loadValue("max_iterations", max_iterations_);
    yaml_loader.loadValue("calc_cost_interval", calc_cost_interval_);
    yaml_loader.loadValue("activation_type", activation_type_);
    yaml_loader.loadVector("layer", neuron_num_);

    bool load_weight_success = true;

    for(unsigned int i = 0; i < neuron_num_.size() - 1; ++i)
    {
      std::stringstream tag;
      tag << "weight" << i;

      Eigen::MatrixXd tmp;
      load_weight_success &= yaml_loader.loadMatrix(tag.str(), tmp);

      w_.push_back(tmp);
    }

    if(thread_num_ == 0)
    {
      std::stringstream msg;
      msg << "thread_num should be > 0." << std::endl
          << "        thread_num : " << thread_num_;

      throw nn::Exception(src, msg.str());
    }
#ifdef _OPENMP
    else if(thread_num_ > omp_get_num_procs())
    {
      std::stringstream msg;
      msg << "thread_num is recommended to be <= the number of processors." << std::endl
          << "        thread_num     : " << thread_num_ << std::endl
          << "        processors num : " << omp_get_num_procs();

      nn::Warning warning(src, msg.str());
      warning.print();
    }
#endif

    if(batch_size_ == 0)
    {
      std::stringstream msg;
      msg << "batch_size should be > 0." << std::endl
          << "        batch_size : " << batch_size_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }
    else if(batch_size_ < thread_num_)
    {
      std::stringstream msg;
      msg << "batch_size should be >= thread_num." << std::endl
          << "        batch_size : " << batch_size_ << std::endl
          << "        thread_num : " << thread_num_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }
    else if(batch_size_ % thread_num_ != 0)
    {
      std::stringstream msg;
      msg << "batch_size should be divisible by thread_num" << std::endl
          << "        batch_size : " << batch_size_ << std::endl
          << "        thread_num : " << thread_num_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(learning_rate_ <= 0.0)
    {
      std::stringstream msg;
      msg << "learning_rate should be > 0." << std::endl
          << "        learning_rate : " << learning_rate_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(momentum_rate_ < 0.0 || 1.0 < momentum_rate_)
    {
      std::stringstream msg;
      msg << "momentum_rate should be > 0 and <= 1.0." << std::endl
          << "        momentum_rate : " << momentum_rate_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(activation_gain_ <= 0.0)
    {
      std::stringstream msg;
      msg << "activation_gain should be > 0.0." << std::endl
          << "        activation_gain : " << activation_gain_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(reference_cost_ <= 0.0)
    {
      std::stringstream msg;
      msg << "reference_cost should be > 0.0." << std::endl
          << "        reference_cost : " << reference_cost_ << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(neuron_num_.size() < 3)
    {
      std::stringstream msg;
      msg << "The number of layer is not enough. It should be > 2." << std::endl
          << "        neuron_num.size() : " << neuron_num_.size() << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(activation_type_ != "sigmoid" &&
       activation_type_ != "sigmoid_table" &&
       activation_type_ != "piecewise_linear" &&
       activation_type_ != "rectified_linear")
    {
      std::stringstream msg;
      msg << "activation_type is not appropriate." << std::endl
          << "        activation_type : " << activation_type_ << std::endl
          << "        It should be one of the follows." << std::endl
          << "        \"sigmoid\"" << std::endl
          << "        \"sigmoid_table\"" << std::endl
          << "        \"piecewise_linear\"" << std::endl
          << "        \"rectified_linear\"" << std::endl
          << "        Please modify \"" << yaml_name << "\"";

      throw nn::Exception(src, msg.str());
    }

    if(!enable_back_propagation_ && !load_weight_success)
    {
      std::stringstream msg;
      msg << "Could not load weight values and enable_back_propagation is \"false\"." << std::endl
          << "        Please make enable_back_propagation \"true\" or" << std::endl
          << "        use yaml file including weight values.";

      throw nn::Exception(src, msg.str());
    }
  }
  catch(std_utils::Exception& e)
  {
    throw nn::Exception("nn::Config::init", e.what());
  }
}

void Config::print()
{
  std::cout << "enable_back_propagation : " << enable_back_propagation_ << std::endl
            << "thread_num : "         << thread_num_         << std::endl
            << "batch_size : "         << batch_size_         << std::endl
            << "learning_rate : "      << learning_rate_      << std::endl
            << "momentum_rate : "      << momentum_rate_      << std::endl
            << "activation_gain : "    << activation_gain_    << std::endl
            << "reference_cost : "     << reference_cost_     << std::endl
            << "max_iterations : "     << max_iterations_     << std::endl
            << "calc_cost_interval : " << calc_cost_interval_ << std::endl
            << "activation_type : "    << activation_type_    << std::endl
            << "layer num       : "    << neuron_num_.size()  << std::endl;

  for(unsigned int i = 0; i < neuron_num_.size(); ++i)
  {
    std::cout << "layer[" << i << "]'s neuron_num : " << neuron_num_[i] << std::endl;
  }
}
