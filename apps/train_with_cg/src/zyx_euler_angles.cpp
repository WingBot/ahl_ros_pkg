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

#include "train_with_cg/zyx_euler_angles.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

ZYXEulerAngles::ZYXEulerAngles(const std::vector<double>& max, const std::vector<double>& min, double step)
  : max_(max), min_(min), step_(step)
{
  euler_ = Eigen::MatrixXd::Zero(3, 1);

  if(max.size() != euler_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"max\" vector is wrong." << std::endl
        << "        size : " << max.size();

    throw train::Exception("ZYXEulerAngles::ZYXEulerAngles", msg.str());
  }

  if(min.size() != euler_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"min\" vector is wrong." << std::endl
        << "        size : " << min.size();

    throw train::Exception("ZYXEulerAngles::ZYXEulerAngles", msg.str());
  }

  for(unsigned int i = 0; i < min.size(); ++i)
  {
    euler_.coeffRef(i, 0) = min[i];
  }

  for(unsigned int i = 0; i < euler_.rows(); ++i)
  {
    step_size_.push_back(static_cast<unsigned int>((max_[i] - min_[i])/step_));
    step_idx_.push_back(0);
  }
}

bool ZYXEulerAngles::isQuaternion()
{
  return false;
}

bool ZYXEulerAngles::isSameAs(const OrientationPtr& orientation, double threshold)
{
  double diff_x = euler_.coeff(0, 0) - orientation->getOrientation().coeff(0, 0);
  double diff_y = euler_.coeff(1, 0) - orientation->getOrientation().coeff(1, 0);
  double diff_z = euler_.coeff(2, 0) - orientation->getOrientation().coeff(2, 0);

  bool is_same = true;

  if(std::fabs(diff_x) > threshold)
    is_same = false;

  if(std::fabs(diff_y) > threshold)
    is_same = false;

  if(std::fabs(diff_z) > threshold)
    is_same = false;

  return is_same;
}

bool ZYXEulerAngles::update()
{
  if(this->isLast())
    return false;



  return true;
}

bool ZYXEulerAngles::isLast()
{
  return false;
}

void ZYXEulerAngles::set(const Eigen::MatrixXd& orientation)
{
  if(orientation.rows() != 3)
  {
    std::stringstream msg;
    msg << "The size of orientation matrix is wrong." << std::endl
        << "        rows : " << orientation.rows();

    throw train::Exception("ZYXEulerAngles::set", msg.str());
  }

  euler_ = orientation;
}

void ZYXEulerAngles::set(double euler_x, double euler_y, double euler_z)
{
  euler_.coeffRef(0, 0) = euler_x;
  euler_.coeffRef(1, 0) = euler_y;
  euler_.coeffRef(2, 0) = euler_z;
}

const Eigen::MatrixXd& ZYXEulerAngles::getOrientation() const
{
  return euler_;
}
