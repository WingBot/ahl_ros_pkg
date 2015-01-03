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

#include <GL/gl.h>
#include <GL/glut.h>
#include <gl_wrapper/render/camera.hpp>
#include <gl_wrapper/render/quaternion.hpp>

using namespace gl_wrapper;

Camera::Camera(
  bool p, double ff, double z_nn, double z_ff,
  const Eigen::Vector3d& pp, const Eigen::Vector3d& cc, const Eigen::Vector3d& uu,
  double zoom_rate, double translate_rate, double rotate_rate)
  : perspective_(p), fovy_(ff), z_near_(z_nn), z_far_(z_ff),
    left_(-1.0), right_(1.0), bottom_(-1.0), top_(1.0),
    pos_(pp), center_(cc), up_(uu),
    pre_pos_(pp), pre_center_(cc), pre_up_(cc),
    init_pos_(pp), init_center_(cc), init_up_(uu),
    zoom_rate_(zoom_rate), translate_rate_(translate_rate), rotate_rate_(rotate_rate)
{
}

void Camera::look(int w, int h)
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  glViewport(0, 0, w, h);

  if(perspective_)
  {
    gluPerspective(fovy_, static_cast<double>(w)/h, z_near_, z_far_);
  }
  else
  {
    this->calcOrthoParams(w, h);
    glOrtho(left_, right_, bottom_, top_, z_near_, z_far_);
  }

  gluLookAt(
    pos_.coeff(0), pos_.coeff(1), pos_.coeff(2),
    center_.coeff(0), center_.coeff(1), center_.coeff(2),
    up_.coeff(0), up_.coeff(1), up_.coeff(2));
}

void Camera::reset()
{
  pos_    = pre_pos_    = init_pos_;
  center_ = pre_center_ = init_center_;
  up_     = pre_up_     = init_up_;
}

void Camera::zoomin()
{
  Eigen::Vector3d temp = pos_ - center_;

  temp *= 1.0 - zoom_rate_;
  pos_ = temp + center_;
}

void Camera::zoomout()
{
  Eigen::Vector3d temp = pos_ - center_;

  temp *= 1.0 + zoom_rate_;
  pos_ = temp + center_;
}

void Camera::translate(int dx, int dy)
{
  Eigen::Vector3d roll_axis = (pos_ - center_).cross(up_);
  roll_axis /= roll_axis.norm();

  double rate_ = (pos_ - center_).norm() * translate_rate_;

  pos_    += dx * rate_ * roll_axis;
  center_ += dx * rate_ * roll_axis;

  Eigen::Vector3d yaw_axis = roll_axis.cross(pos_ - center_);
  yaw_axis /= yaw_axis.norm();

  pos_    += dy * rate_ * yaw_axis;
  center_ += dy * rate_ * yaw_axis;
}

void Camera::rotate(int dx, int dy)
{
  Eigen::Vector3d temp_p = pos_ - center_;
  Eigen::Vector3d roll_axis = temp_p.cross(up_);
  roll_axis /= roll_axis.norm();

  Quaternion q_eye(temp_p);
  Quaternion q_up(up_);

  double roll = -dy * rotate_rate_;
  Quaternion q_roll(roll_axis, roll);

  q_eye.rotateWith(q_roll);
  q_up.rotateWith(q_roll);

  double yaw = dx * rotate_rate_;
  Eigen::Vector3d yaw_axis;
  yaw_axis << q_up.getX(), q_up.getY(), q_up.getZ();
  Quaternion q_yaw(yaw_axis, yaw);

  q_eye.rotateWith(q_yaw);

  pos_.coeffRef(0) = q_eye.getX() + center_.coeff(0);
  pos_.coeffRef(1) = q_eye.getY() + center_.coeff(1);
  pos_.coeffRef(2) = q_eye.getZ() + center_.coeff(2);

  up_.coeffRef(0) = q_up.getX();
  up_.coeffRef(1) = q_up.getY();
  up_.coeffRef(2) = q_up.getZ();
}

void Camera::calcOrthoParams(int w, int h)
{
  double norm   = (pos_ - center_).norm();
  double d = 2.0 * atan(fovy_ * 0.5) * norm;
  double view_h = d;
  double view_w = d;

  if(h < w)
  {
    view_h = view_w * (static_cast<double>(h) / w);
  }
  else
  {
    view_w = view_h * (static_cast<double>(w) / h);
  }

  right_  = view_w * 0.5;
  left_   = -right_;
  top_    = view_h * 0.5;
  bottom_ = -top_;
}

