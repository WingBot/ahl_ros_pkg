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

#include <gl_wrapper/render/material.hpp>

using namespace gl_wrapper;

Material::Material(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine)
{
  for(int i = 0; i < 4; ++i)
  {
    ambient_[i]  = amb[i];
    diffuse_[i]  = dif[i];
    specular_[i] = spe[i];
    shine_       = shine;
  }
}

void Material::setMaterial(GLfloat* amb, GLfloat* dif, GLfloat* spe, GLfloat shine)
{
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   amb);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   dif);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  spe);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, shine);
}

void Material::apply()
{
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   ambient_);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   diffuse_);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_);
  glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, shine_);
}
