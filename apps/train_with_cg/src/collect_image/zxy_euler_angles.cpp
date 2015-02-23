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

#include "train_with_cg/collect_image/zxy_euler_angles.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

ZXYEulerAngles::ZXYEulerAngles(const std::vector<double>& max, const std::vector<double>& min, double step)
  : step_(step)
{
  euler_ = Eigen::MatrixXd::Zero(3, 1);
  max_.resize(euler_.rows(), euler_.cols());
  min_.resize(euler_.rows(), euler_.cols());

  if(max.size() != euler_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"max\" vector is wrong." << std::endl
        << "        size : " << max.size();

    throw train::Exception("ZXYEulerAngles::ZXYEulerAngles", msg.str());
  }

  if(min.size() != euler_.rows())
  {
    std::stringstream msg;
    msg << "The size of \"min\" vector is wrong." << std::endl
        << "        size : " << min.size();

    throw train::Exception("ZXYEulerAngles::ZXYEulerAngles", msg.str());
  }

  for(unsigned int i = 0; i < euler_.rows(); ++i)
  {
    euler_.coeffRef(i, 0) = min[i];
    max_.coeffRef(i, 0)   = max[i];
    min_.coeffRef(i, 0)   = min[i];
  }

  for(unsigned int i = 0; i < euler_.rows(); ++i)
  {
    step_size_.push_back(static_cast<unsigned int>((max_.coeff(i, 0) - min_.coeff(i, 0))/step_));
    step_idx_.push_back(0);
  }
}

bool ZXYEulerAngles::isQuaternion()
{
  return false;
}

bool ZXYEulerAngles::isSameAs(const OrientationPtr& orientation, double threshold)
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

bool ZXYEulerAngles::update()
{
  if(this->updateX() == false)
  {
    if(this->updateY() == false)
    {
      if(this->updateZ() == false)
      {
        return false;
      }
      else
      {
        this->resetX();
        this->resetY();
      }
    }
    else
    {
      this->resetX();
    }
  }

  return true;
}

void ZXYEulerAngles::reset()
{
  euler_ = min_;
}

void ZXYEulerAngles::set(const Eigen::MatrixXd& orientation)
{
  if(orientation.rows() != 3)
  {
    std::stringstream msg;
    msg << "The size of orientation matrix is wrong." << std::endl
        << "        rows : " << orientation.rows();

    throw train::Exception("ZXYEulerAngles::set", msg.str());
  }

  euler_ = orientation;
}

void ZXYEulerAngles::set(double euler_x, double euler_y, double euler_z)
{
  euler_.coeffRef(0, 0) = euler_x;
  euler_.coeffRef(1, 0) = euler_y;
  euler_.coeffRef(2, 0) = euler_z;
}

const Eigen::MatrixXd& ZXYEulerAngles::getOrientation() const
{
  return euler_;
}

bool ZXYEulerAngles::updateX()
{
  ++step_idx_[0];
  if(step_idx_[0] > step_size_[0])
    return false;

  euler_.coeffRef(0, 0) = min_.coeff(0, 0) + step_idx_[0] * step_;

  return true;
}

bool ZXYEulerAngles::updateY()
{
  ++step_idx_[1];
  if(step_idx_[1] > step_size_[1])
    return false;

  euler_.coeffRef(1, 0) = min_.coeff(1, 0) + step_idx_[1] * step_;
 
  return true;
}

bool ZXYEulerAngles::updateZ()
{
  ++step_idx_[2];
  if(step_idx_[2] > step_size_[2])
    return false;

  euler_.coeffRef(2, 0) = min_.coeff(2, 0) + step_idx_[2] * step_;

  return true;
}

void ZXYEulerAngles::resetX()
{
  euler_.coeffRef(0, 0) = min_.coeff(0, 0);
  step_idx_[0] = 0;
}

void ZXYEulerAngles::resetY()
{
  euler_.coeffRef(1, 0) = min_.coeff(1, 0);
  step_idx_[1] = 0;
}

void ZXYEulerAngles::resetZ()
{
  euler_.coeffRef(2, 0) = min_.coeff(2, 0);
  step_idx_[2] = 0;
}
