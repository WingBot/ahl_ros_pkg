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

#ifndef __GL_WRAPPER_RENDER_PARAM_HPP
#define __GL_WRAPPER_RENDER_PARAM_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <GL/gl.h>
#include <Eigen/Dense>
#include <gl_wrapper/exception/exceptions.hpp>

namespace gl_wrapper
{

  class Param
  {
  public:
    Param();

    boost::mutex mutex;

    std::string window_name;
    int window_h;
    int window_w;
    std::vector<int> color;

    double fps;

    bool orthogonal;
    double fovy;
    double z_near;
    double z_far;

    Eigen::Vector3d camera_pos;
    Eigen::Vector3d camera_center;
    Eigen::Vector3d camera_up;

    Eigen::Vector3d init_camera_pos;
    Eigen::Vector3d init_camera_center;
    Eigen::Vector3d init_camera_up;

    double zoom_rate;
    double translate_rate;
    double rotate_rate;

    int light_num;
    std::vector<Eigen::Vector4d> light_pos;
    std::vector<Eigen::Vector4d> ambient;
    std::vector<Eigen::Vector4d> diffuse;
    std::vector<Eigen::Vector4d> specular;

  private:
    template<class T>
    void checkLowerBorder(T val, T min, const std::string& name)
    {
      if(val < min)
      {
        std::stringstream msg;
        msg << name << "(= " << val << ") should be larger than " << min << ".";
        throw Exception("Param::Param", msg.str());
      }      
    }
    template<class T>
    void checkUpperBorder(T val, T max, const std::string& name)
    {
      if(val > max)
      {
        std::stringstream msg;
        msg << name << "(= " << val << ") should be smaller than " << max << ".";
        throw Exception("Param::Param", msg.str());
      }      
    }
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __GL_WRAPPER_RENDER_PARAM_HPP */
