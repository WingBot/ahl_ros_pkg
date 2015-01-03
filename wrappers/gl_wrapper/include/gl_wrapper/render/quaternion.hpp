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

#ifndef __GL_WRAPPER_MATH_QUATERNION_HPP
#define __GL_WRAPPER_MATH_QUATERNION_HPP

#include <Eigen/Dense>
#include <gl_wrapper/exception/exceptions.hpp>

namespace gl_wrapper
{

  class Quaternion
  {
  public:
    Quaternion();
    Quaternion(double x, double y, double z, double w);
    Quaternion(const Eigen::Vector3d& axis, double angle);
    Quaternion(const Eigen::Vector3d& pos);

    Quaternion& operator=(const Quaternion& q);
    void operator*=(Quaternion& q);
    const Quaternion operator*(Quaternion& q) const;

    Quaternion inverse();
    Quaternion& rotateWith(Quaternion& q);

    const double getNorm() const
    {
      return sqrt(x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_);
    }

    const double getX() const
    {
      return x_;
    }

    const double getY() const
    {
      return y_;
    }

    const double getZ() const
    {
      return z_;
    }

    const double getW() const
    {
      return w_;
    }

  private:
    double x_;
    double y_;
    double z_;
    double w_;
  };

}

#endif /* __GL_WRAPPER_MATH_QUATERNION_HPP */
