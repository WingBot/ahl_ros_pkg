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

#include <GL/glut.h>
#include <gl_wrapper/object/coordinate.hpp>

using namespace gl_wrapper;

Coordinate::Coordinate(double scale, double length)
  : scale_(scale), length_(length)
{

}

void Coordinate::displayImpl()
{
  glScaled(scale_, scale_, scale_);
  glDisable(GL_LIGHTING);

  const double radius = 0.03;

  glColor3d(1.0, 0.0, 0.0);
  glRotated(90.0, 0.0, 1.0, 0.0);
  glutSolidCylinder(radius, length_, 16, 16);

  glColor3d(0.0, 1.0, 0.0);
  glRotated(-90.0, 0.0, 1.0, 0.0);
  glRotated(-90.0, 1.0, 0.0, 0.0);
  glutSolidCylinder(radius, length_, 16, 16);

  glColor3d(0.0, 0.0, 1.0);
  glRotated(90.0, 1.0, 0.0, 0.0);
  glTranslated(0.0, 0.0, -radius);
  glutSolidCylinder(radius, length_ + radius, 16, 16);

  glEnable(GL_LIGHTING);
  double scale_inv = 1.0/scale_;
  glScaled(scale_inv, scale_inv, scale_inv);
}
