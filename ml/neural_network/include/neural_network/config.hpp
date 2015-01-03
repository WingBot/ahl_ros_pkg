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

#ifndef __NEURAL_NETWORK_CONFIG_HPP
#define __NEURAL_NETWORK_CONFIG_HPP

#include <iostream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace nn
{

  class Config
  {
  public:
    Config();

    void init(const std::string& yaml_name);

    const bool enableBackPropagation() const
    {
      return enable_back_propagation_;
    }

    const unsigned int getThreadNum() const
    {
      return thread_num_;
    }

    const unsigned int getBatchSize() const
    {
      return batch_size_;
    }

    const double getLearningRate() const
    {
      return learning_rate_;
    }

    const double getMomentumRate() const
    {
      return momentum_rate_;
    }

    const double getActivationGain() const
    {
      return activation_gain_;
    }

    const double getReferenceCost() const
    {
      return reference_cost_;
    }

    const unsigned long getMaxIterations() const
    {
      return max_iterations_;
    }

    const unsigned long getCalcCostInterval() const
    {
      return calc_cost_interval_;
    }

    const std::string& getActivationType() const
    {
      return activation_type_;
    }

    const std::vector<unsigned int>& getNeuronNum() const
    {
      return neuron_num_;
    }

    const std::vector<Eigen::MatrixXd>& getWVector() const
    {
      return w_;
    }

    void print();

  private:
    template<class T>
    void printWarning(const std::string& yaml_name, const std::string& tag, T default_value)
    {
      std::cout << yaml_name << " doesn't have \"" << tag << "\" tag." << std::endl
                << "Default value (" << default_value << ") is used." << std::endl;
    }

    bool enable_back_propagation_;
    unsigned int thread_num_;
    unsigned int batch_size_;
    double learning_rate_;
    double momentum_rate_;
    double activation_gain_;
    double reference_cost_; // If cost is smaller than reference_cost_, training will be finished.
    unsigned long max_iterations_; // If number of iteration is bigger than max_iterations_, training will be finished.
    unsigned long calc_cost_interval_; // Calculate cost per calc_cost_interval_ iterations.
    std::vector<unsigned int> neuron_num_; // Doesn't include neuron for "threshold term". Vector size means layer num, including input and output layers.

    std::string activation_type_;
    std::vector<Eigen::MatrixXd> w_;
  };

  typedef boost::shared_ptr<Config> ConfigPtr;
}

#endif /* __NEURAL_NETWORK_CONFIG_HPP */
