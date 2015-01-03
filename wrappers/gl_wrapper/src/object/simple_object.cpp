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

#include <gl_wrapper/object/simple_object.hpp>

using namespace gl_wrapper;

SimpleObject::SimpleObject()
  : pos_(Eigen::Vector3d::Zero()), euler_zyx_(Eigen::Vector3d::Zero()), r_(0), g_(0), b_(0)
{
}

void SimpleObject::display()
{
  glLoadIdentity();
  glPushMatrix();

  glTranslated(pos_.coeff(0), pos_.coeff(1), pos_.coeff(2));
  glRotated(euler_zyx_.coeff(0), 0.0, 0.0, 1.0);
  glRotated(euler_zyx_.coeff(1), 0.0, 1.0, 0.0);
  glRotated(euler_zyx_.coeff(2), 1.0, 0.0, 0.0);

  glColor3d(static_cast<double>(r_ / 255.0), static_cast<double>(g_ / 255.0), static_cast<double>(b_ / 255.0));
  this->displayImpl();

  glPopMatrix();
}

void SimpleObject::setColor(int r, int g, int b)
{
  r_ = r;
  g_ = g;
  b_ = b;
}

void SimpleObject::setPosition(const Eigen::Vector3d& pos)
{
  pos_ = pos;
}

void SimpleObject::setPosition(double x, double y, double z)
{
  pos_.coeffRef(0) = x;
  pos_.coeffRef(1) = y;
  pos_.coeffRef(2) = z;
}

void SimpleObject::setEulerZYX(const Eigen::Vector3d& euler_zyx)
{
  euler_zyx_ = euler_zyx;
}

void SimpleObject::setEulerZYX(double rz, double ry, double rx)
{
  euler_zyx_.coeffRef(0) = rz;
  euler_zyx_.coeffRef(1) = ry;
  euler_zyx_.coeffRef(2) = rx;
}
