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

#ifndef __GL_WRAPPER_OBJECT_SIMPLE_OBJECT_HPP
#define __GL_WRAPPER_OBJECT_SIMPLE_OBJECT_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

namespace gl_wrapper
{

  class SimpleObject
  {
  public:
    SimpleObject();
    virtual ~SimpleObject() {}
    virtual void display();
    virtual void setColor(int r, int g, int b);
    virtual void setPosition(const Eigen::Vector3d& pos);
    virtual void setPosition(double x, double y, double z);
    virtual void setEulerZYX(const Eigen::Vector3d& euler_zyx);
    virtual void setEulerZYX(double rz, double ry, double rx);

  private:
    virtual void displayImpl() = 0;

    Eigen::Vector3d pos_;
    Eigen::Vector3d euler_zyx_;

    int r_;
    int g_;
    int b_;
  };

  typedef boost::shared_ptr<SimpleObject> SimpleObjectPtr;
}

#endif /* __GL_WRAPPER_OBJECT_SIMPLE_OBJECT_HPP */
