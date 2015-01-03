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

#include <ros/ros.h>
#include <gl_wrapper/render/param.hpp>

using namespace gl_wrapper;

Param::Param()
{
  ros::NodeHandle local_nh("/gl_wrapper");
  color.resize(3);

  local_nh.param<std::string>("window/name", window_name, "OpenGL");
  local_nh.param<int>("window/h", window_h, 960);
  local_nh.param<int>("window/w", window_w, 1280);
  local_nh.param<int>("window/r", color[0], 255);
  local_nh.param<int>("window/g", color[1], 255);
  local_nh.param<int>("window/b", color[2], 255);

  this->checkLowerBorder(window_h, 0, "window_h");
  this->checkLowerBorder(window_w, 0, "window_w");
  this->checkLowerBorder(color[0], 0, "color[0]");
  this->checkLowerBorder(color[1], 0, "color[1]");
  this->checkLowerBorder(color[2], 0, "color[2]");
  this->checkUpperBorder(color[0], 255, "color[0]");
  this->checkUpperBorder(color[1], 255, "color[1]");
  this->checkUpperBorder(color[2], 255, "color[2]");

  local_nh.param<double>("fps", fps, 33.333333);
  this->checkLowerBorder(fps, 0.0, "fps");

  local_nh.param<bool>("orthogonal", orthogonal, false);
  local_nh.param<double>("fovy", fovy, 60.0);
  local_nh.param<double>("z_near", z_near, 0.1);
  local_nh.param<double>("z_far", z_far, 100.0);

  this->checkLowerBorder(fovy, 0.0, "fovy");
  this->checkLowerBorder(z_near, 0.0, "z_near");
  this->checkLowerBorder(z_far, 0.0, "z_far");
  this->checkUpperBorder(fovy, 180.0, "fovy");

  local_nh.param<double>("camera/pos/x", camera_pos.coeffRef(0), 3.0);
  local_nh.param<double>("camera/pos/y", camera_pos.coeffRef(1), 0.0);
  local_nh.param<double>("camera/pos/z", camera_pos.coeffRef(2), 0.0);
  local_nh.param<double>("camera/center/x", camera_center.coeffRef(0), 0.0);
  local_nh.param<double>("camera/center/y", camera_center.coeffRef(1), 0.0);
  local_nh.param<double>("camera/center/z", camera_center.coeffRef(2), 0.0);
  local_nh.param<double>("camera/up/x", camera_up.coeffRef(0), 0.0);
  local_nh.param<double>("camera/up/y", camera_up.coeffRef(1), 0.0);
  local_nh.param<double>("camera/up/z", camera_up.coeffRef(2), 1.0);

  local_nh.param<double>("camera/pos/init_x", init_camera_pos.coeffRef(0), 3.0);
  local_nh.param<double>("camera/pos/init_y", init_camera_pos.coeffRef(1), 0.0);
  local_nh.param<double>("camera/pos/init_z", init_camera_pos.coeffRef(2), 0.0);
  local_nh.param<double>("camera/center/init_x", init_camera_center.coeffRef(0), 0.0);
  local_nh.param<double>("camera/center/init_y", init_camera_center.coeffRef(1), 0.0);
  local_nh.param<double>("camera/center/init_z", init_camera_center.coeffRef(2), 0.0);
  local_nh.param<double>("camera/up/init_x", init_camera_up.coeffRef(0), 0.0);
  local_nh.param<double>("camera/up/init_y", init_camera_up.coeffRef(1), 0.0);
  local_nh.param<double>("camera/up/init_z", init_camera_up.coeffRef(2), 1.0);

  local_nh.param<double>("camera/zoom_rate", zoom_rate, 0.1);
  local_nh.param<double>("camera/translate_rate", translate_rate, 0.0008);
  local_nh.param<double>("camera/rotate_rate", rotate_rate, 0.01);

  this->checkLowerBorder(zoom_rate, 0.0, "zoom_rate");
  this->checkLowerBorder(translate_rate, 0.0, "translate_rate");
  this->checkLowerBorder(rotate_rate, 0.0, "rotate_rate");

  int light_num_max;
  glGetIntegerv(GL_MAX_LIGHTS, &light_num_max);
  local_nh.param<int>("light/num", light_num, 1);

  this->checkLowerBorder(light_num, 0, "light_num");
  this->checkUpperBorder(light_num, light_num_max, "light_num");

  if(light_num > 0)
  {
    light_pos.resize(light_num);
    ambient.resize(light_num);
    diffuse.resize(light_num);
    specular.resize(light_num);

    for(int i = 0; i < light_num; ++i)
    {
      std::stringstream light_str;
      light_str << "light" << i;

      local_nh.param<double>(light_str.str() + "/pos/x", light_pos[i].coeffRef(0), 1.f);
      local_nh.param<double>(light_str.str() + "/pos/y", light_pos[i].coeffRef(1), 1.f);
      local_nh.param<double>(light_str.str() + "/pos/z", light_pos[i].coeffRef(2), 1.f);
      local_nh.param<double>(light_str.str() + "/pos/w", light_pos[i].coeffRef(3), 0.f);

      local_nh.param<double>(light_str.str() + "/ambient/r", ambient[i].coeffRef(0), 0.f);
      local_nh.param<double>(light_str.str() + "/ambient/g", ambient[i].coeffRef(1), 0.f);
      local_nh.param<double>(light_str.str() + "/ambient/b", ambient[i].coeffRef(2), 0.f);
      local_nh.param<double>(light_str.str() + "/ambient/a", ambient[i].coeffRef(3), 1.f);

      local_nh.param<double>(light_str.str() + "/diffuse/r", diffuse[i].coeffRef(0), 1.f);
      local_nh.param<double>(light_str.str() + "/diffuse/g", diffuse[i].coeffRef(1), 1.f);
      local_nh.param<double>(light_str.str() + "/diffuse/b", diffuse[i].coeffRef(2), 1.f);
      local_nh.param<double>(light_str.str() + "/diffuse/a", diffuse[i].coeffRef(3), 1.f);

      local_nh.param<double>(light_str.str() + "/specular/r", specular[i].coeffRef(0), 1.f);
      local_nh.param<double>(light_str.str() + "/specular/g", specular[i].coeffRef(1), 1.f);
      local_nh.param<double>(light_str.str() + "/specular/b", specular[i].coeffRef(2), 1.f);
      local_nh.param<double>(light_str.str() + "/specular/a", specular[i].coeffRef(3), 1.f);
    }
  }
}

