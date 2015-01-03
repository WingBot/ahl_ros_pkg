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

#ifndef __GL_WRAPPER_OBJECT_OBJECT_HPP
#define __GL_WRAPPER_OBJECT_OBJECT_HPP

#include <vector>
#include <GL/glut.h>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <gl_wrapper/render/material.hpp>

namespace gl_wrapper
{
  class Object;
  typedef boost::shared_ptr<Object> ObjectPtr;

  class Object
  {
  public:
    Object();

    void display();
    bool haveEmptyParam();

    bool assign_normal_to_vertex; // If false, assign normal to face
    bool assign_material_to_vertex; // If false, assign material to face

    class Offset
    {
    public:
      Offset() : euler_zyx(Eigen::Vector3d::Zero()), translation(Eigen::Vector3d::Zero()) {}
      Eigen::Vector3d euler_zyx;
      Eigen::Vector3d translation;
    };

    class Motion
    {
    public:
      Motion() : euler_zyx(Eigen::Vector3d::Zero()), translation(Eigen::Vector3d::Zero()) {}
      Eigen::Vector3d euler_zyx;
      Eigen::Vector3d translation;
    };

    Offset offset;
    Motion motion;

    std::vector<Eigen::Vector3d> vertex;
    std::vector< std::vector<unsigned int> > v_idx;
    std::vector<Eigen::Vector3d> normal;
    std::vector< std::vector<unsigned int> > n_idx;

    std::vector<MaterialPtr> material;
    std::vector<unsigned int> material_assignment;

    std::string name;
    ObjectPtr parent;
    ObjectPtr child;

    GLuint model_list;
  };
}

#endif /* __GL_WRAPPER_OBJECT_OBJECT_HPP */
