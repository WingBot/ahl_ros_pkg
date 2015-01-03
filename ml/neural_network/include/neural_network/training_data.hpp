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

#ifndef __NEURAL_NETWORK_TRAINING_DATA_HPP
#define __NEURAL_NETWORK_TRAINING_DATA_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace nn
{
  class TrainingData;
  typedef boost::shared_ptr<TrainingData> TrainingDataPtr;

  class TrainingData
  {
  public:
    TrainingData();
    void add(const std::vector<double>& input, const std::vector<double>& output);
    void add(const Eigen::MatrixXd& input, const Eigen::MatrixXd& output);
    void init(const std::string& name_in, const std::string& name_out);
    void shuffle();
    void separate(std::vector<TrainingDataPtr>& dst, unsigned int sep_num);
    void separate(std::vector< std::vector<TrainingDataPtr> >& dst, unsigned int thread_num, unsigned int batch_size);

    const std::vector<Eigen::MatrixXd>& getInput() const
    {
      return input_;
    }

    const std::vector<Eigen::MatrixXd>& getOutput() const
    {
      return output_;
    }

    const unsigned int getSize() const
    {
      return input_.size();
    }

    void setInput(unsigned int idx, std::vector<double>& input);
    void setInput(unsigned int idx, Eigen::MatrixXd& input);

    void setOutput(unsigned int idx, std::vector<double>& output);
    void setOutput(unsigned int idx, Eigen::MatrixXd& output);

    void print();

  private:
    std::vector<Eigen::MatrixXd> input_;
    std::vector<Eigen::MatrixXd> output_;
  };
}

#endif /* __NEURAL_NETWORK_TRAINING_DATA_HPP */
