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

#include <gl_wrapper/exception/exceptions.hpp>
#include <gl_wrapper/render/quaternion.hpp>

using namespace gl_wrapper;

Quaternion::Quaternion()
  : x_(0.0), y_(0.0), z_(1.0), w_(0.0)
{
}

Quaternion::Quaternion(double x, double y, double z, double w)
  : x_(x), y_(y), z_(z), w_(w)
{
}

Quaternion::Quaternion(const Eigen::Vector3d& axis, double angle)
{
  if(axis.norm() == 0.0)
  {
    throw Exception("Quaternion::Quaternion", "Norm of axis is zero.");
  }

  double inv_norm = 1.0 / axis.norm();
  double sin_val = sin(0.5 * angle);

  x_ = axis.coeff(0) * inv_norm * sin_val;
  y_ = axis.coeff(1) * inv_norm * sin_val;
  z_ = axis.coeff(2) * inv_norm * sin_val;
  w_ = cos(0.5 * angle);
}

Quaternion::Quaternion(const Eigen::Vector3d& pos)
  : x_(pos.coeff(0)), y_(pos.coeff(1)), z_(pos.coeff(2)), w_(0.0)
{
}

Quaternion& Quaternion::operator=(const Quaternion& q)
{
  if(this == &q)
    return *this;

  x_ = q.getX();
  y_ = q.getY();
  z_ = q.getZ();
  w_ = q.getW();

  return *this;
}

void Quaternion::operator*=(Quaternion& q)
{
  double x = x_;
  double y = y_;
  double z = z_;
  double w = w_;

  w_ = w * q.getW() - x * q.getX() - y * q.getY() - z * q.getZ();
  x_ = w * q.getX() + x * q.getW() + y * q.getZ() - z * q.getY();
  y_ = w * q.getY() + y * q.getW() + z * q.getX() - x * q.getZ();
  z_ = w * q.getZ() + z * q.getW() + x * q.getY() - y * q.getX();
}

const Quaternion Quaternion::operator*(Quaternion& q) const
{
  Quaternion dst(*this);
  dst *= q;

  return dst;
}

Quaternion Quaternion::inverse()
{
  double norm = this->getNorm();
  if(norm == 0.0)
  {
    throw Exception("Quaternion::inverse", "Could not calculate inverse because the norm is zero.");
  }

  double inv_norm = 1.0 / norm;

  Quaternion q(-x_ * inv_norm, -y_ * inv_norm, -z_ * inv_norm, w_ * inv_norm);
  return q;
}

Quaternion& Quaternion::rotateWith(Quaternion& q)
{
  *this = q.inverse() * (*this) * q;
  return *this;
}
