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

#include "train_with_cg/fingers.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

Fingers::Fingers(const std::vector<double>& max, const std::vector<double>& min, double step)
  : step_(step)
{
  angles_.resize(5, 1);
  max_.resize(angles_.rows(), angles_.cols());
  min_.resize(angles_.rows(), angles_.cols());

  if(max.size() != angles_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"max\" vector is wrong." << std::endl
        << "        size : " << max.size();

    throw train::Exception("Fingers::Fingers", msg.str());
  }

  if(min.size() != angles_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"min\" vector is wrong." << std::endl
        << "        size : " << min.size();

    throw train::Exception("Fingers::Fingers", msg.str());
  }

  for(unsigned int i = 0; i < angles_.rows(); ++i)
  {
    angles_.coeffRef(i, 0) = min[i];
    max_.coeffRef(i, 0)    = max[i];
    min_.coeffRef(i, 0)    = min[i];

    step_size_.push_back((max_.coeff(i, 0) - min_.coeff(i, 0)) / step_);
    step_idx_.push_back(0);
  }
}

void Fingers::isSameAs(const FingersPtr& fingers)
{

}

void Fingers::set(const Eigen::MatrixXd& angles)
{
  angles_ = angles;
}

void Fingers::set(double angle0, double angle1, double angle2, double angle3, double angle4)
{

}

bool Fingers::update()
{
  return this->updateAll();
}

void Fingers::reset()
{
  for(unsigned int i = 0; i < angles_.rows(); ++i)
  {
    this->reset(i);
  }
}

bool Fingers::updateAll()
{
  bool updated = true;

  for(unsigned int i = 0; i < angles_.rows(); ++i)
  {
    updated &= this->update(i);
  }

  return updated;
}

bool Fingers::update(unsigned int idx)
{
  if(step_idx_[idx] == step_size_[idx])
    return false;

  angles_.coeffRef(idx, 0) = min_.coeffRef(idx, 0) + step_idx_[idx] * step_;
  ++step_idx_[idx];

  return true;
}

void Fingers::reset(unsigned int idx)
{
  angles_.coeffRef(idx, 0) = min_.coeffRef(idx, 0);
  step_idx_[idx] = 0;
}
