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

#include "neural_network/exceptions.hpp"
#include "neural_network/core/multi_threaded_back_propagation.hpp"
#include <std_utils/io_utils.hpp>

using namespace nn;

MultiThreadedBackPropagation::MultiThreadedBackPropagation(const ConfigPtr& config)
  : config_(config)
{
  for(unsigned int i = 0; i < config_->getThreadNum(); ++i)
  {
    thread_.push_back(ThreadPtr(new boost::thread));
    mutex_.push_back(MutexPtr(new boost::mutex));
    condition_.push_back(ConditionPtr(new boost::condition));
    back_propagation_.push_back(BackPropagationPtr(new BackPropagation(config)));
  }

  for(unsigned int i = 0; i < config->getNeuronNum().size(); ++i)
  {
    layer_.push_back(LayerPtr(new Layer(config->getNeuronNum()[i])));
  }

  dw_.resize     (config->getNeuronNum().size() - 1);
  pre_dw_.resize (config->getNeuronNum().size() - 1);
  dw_zero_.resize(config->getNeuronNum().size() - 1);

  for(unsigned int i = 0; i < config->getNeuronNum().size() - 1; ++i)
  {
    dw_[i]      = Eigen::MatrixXd::Zero(config->getNeuronNum()[i + 1], config->getNeuronNum()[i] + 1);
    pre_dw_[i]  = Eigen::MatrixXd::Zero(config->getNeuronNum()[i + 1], config->getNeuronNum()[i] + 1);
    dw_zero_[i] = Eigen::MatrixXd::Zero(config->getNeuronNum()[i + 1], config->getNeuronNum()[i] + 1);
  }

  for(unsigned int i = 0; i < config->getW().size(); ++i)
  {
    if(config->getW()[i].cols() == layer_[i]->getW().cols() &&
       config->getW()[i].rows() == layer_[i]->getW().rows())
    {
      layer_[i]->getWRef() = config->getW()[i];
    }
  }
}

void MultiThreadedBackPropagation::train(const TrainingDataPtr& data)
{
  if(data->getSize() == 0)
  {
    throw nn::Exception("MultiThreadedBackPropagation::train", "Data size is zero.");
  }

  data->shuffle();
  data->separate(data_, config_->getThreadNum());
  data->separate(sep_data_, config_->getThreadNum(), config_->getBatchSize());

  unsigned int batch_num = this->getBatchNum();
  unsigned long loop_cnt = 0;

  for(loop_cnt = 0; loop_cnt < config_->getMaxIterations(); ++loop_cnt)
  {
    for(unsigned int batch_idx = 0; batch_idx < batch_num; ++batch_idx)
    {
      this->startThreads(batch_idx);
      this->correctWeights();
    }

    this->printCost(loop_cnt, data);
  }

  this->printCost(loop_cnt, data);
  back_propagation_[0]->copyLayerTo(layer_);
}

unsigned int MultiThreadedBackPropagation::getBatchNum()
{
  if(sep_data_.size() == 0)
    return 0;

  return sep_data_[0].size();
}

void MultiThreadedBackPropagation::train(unsigned int idx, BackPropagationPtr& back_propagation, TrainingDataPtr& data)
{
  back_propagation_[idx]->train(data);
}

void MultiThreadedBackPropagation::startThreads(unsigned int batch_idx)
{
  for(unsigned int thread_id = 0; thread_id < config_->getThreadNum(); ++thread_id)
  {
    thread_[thread_id].reset();
    thread_[thread_id] = ThreadPtr(new boost::thread(boost::bind(&MultiThreadedBackPropagation::train,
                                                                 this,
                                                                 thread_id,
                                                                 back_propagation_[thread_id],
                                                                 sep_data_[thread_id][batch_idx])));
  }
  for(unsigned int thread_id = 0; thread_id < config_->getThreadNum(); ++thread_id)
  {
    thread_[thread_id]->join();
  }
}

void MultiThreadedBackPropagation::correctWeights()
{
  unsigned int data_size = 0;

  for(unsigned int i = 0; i < config_->getThreadNum(); ++i)
  {
    for(unsigned int j = 0; j < config_->getNeuronNum().size() - 1; ++j)
    {
      dw_[j] += back_propagation_[i]->getDw()[j];
    }

    data_size += back_propagation_[i]->getDataSize();
    back_propagation_[i]->initDw();
  }

  for(unsigned int i = 0; i < config_->getThreadNum(); ++i)
  {
    double coeff = 1.0 / data_size;

    for(unsigned int j = 0; j < config_->getNeuronNum().size() - 1; ++j)
    {
      back_propagation_[i]->correctWeight(j, coeff * dw_[j] + config_->getMomentumRate() * pre_dw_[j]);
    }
  }

  for(unsigned int i = 0; i < dw_.size(); ++i)
  {
    pre_dw_[i] = dw_[i];
    dw_[i] = dw_zero_[i];
  }
}

void MultiThreadedBackPropagation::printCost(unsigned long loop_cnt, const TrainingDataPtr& data)
{
  if((loop_cnt % config_->getCalcCostInterval()) != 0)
    return;

  double cost = 0.0;

  //for(unsigned int i = 0; i < config_->getThreadNum(); ++i)
  //{
  //cost += back_propagation_[i]->getCost();
  //}

  cost = back_propagation_[0]->getCost(data);

  std::cout << loop_cnt
            << "th training result : cost = "
            << cost / config_->getThreadNum()
            << std::endl;
}

