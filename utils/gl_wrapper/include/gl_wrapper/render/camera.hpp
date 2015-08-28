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

#ifndef __GL_WRAPPER_RENDER_CAMERA_HPP
#define __GL_WRAPPER_RENDER_CAMERA_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace gl_wrapper
{

  class Camera
  {
  public:
    Camera(
      bool p, double fovy, double z_near, double z_far,
      const Eigen::Vector3d& pp, const Eigen::Vector3d& cc, const Eigen::Vector3d& uu,
      double zoom_rate, double translate_rate, double rotate_rate);
    void look(int w, int h);
    void reset();

    void zoomin();
    void zoomout();
    void translate(int dx, int dy);
    void rotate(int dx, int dy);

    const Eigen::Vector3d& getPos() const
    {
      return pos_;
    }

    const Eigen::Vector3d& getCenter() const
    {
      return center_;
    }

    const Eigen::Vector3d& getUp() const
    {
      return up_;
    }

  private:
    void calcOrthoParams(int w, int h);

    bool perspective_;

    double fovy_;
    double z_near_;
    double z_far_;

    double left_;
    double right_;
    double bottom_;
    double top_;

    Eigen::Vector3d pos_;
    Eigen::Vector3d center_;
    Eigen::Vector3d up_;

    Eigen::Vector3d pre_pos_;
    Eigen::Vector3d pre_center_;
    Eigen::Vector3d pre_up_;

    Eigen::Vector3d init_pos_;
    Eigen::Vector3d init_center_;
    Eigen::Vector3d init_up_;

    double zoom_rate_;
    double translate_rate_;
    double rotate_rate_;
  };

  typedef boost::shared_ptr<Camera> CameraPtr;
}

#endif /* __GL_WRAPPER_RENDER_CAMERA_HPP */
