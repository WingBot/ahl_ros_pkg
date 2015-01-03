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

#include <gl_wrapper/object/object.hpp>
#include <gl_wrapper/exception/exceptions.hpp>

using namespace gl_wrapper;

Object::Object()
  : assign_normal_to_vertex(true), assign_material_to_vertex(true)
{

}

void Object::display()
{
  if(this->haveEmptyParam())
  {
    throw Exception("Object::display", "Could not display because the object has empty params.");
  }

  glTranslated(offset.translation.coeff(0),
               offset.translation.coeff(1),
               offset.translation.coeff(2));

  glRotated(offset.euler_zyx.coeff(0) * 180.0 / M_PI, 0.0, 0.0, 1.0);
  glRotated(offset.euler_zyx.coeff(1) * 180.0 / M_PI, 0.0, 1.0, 0.0);
  glRotated(offset.euler_zyx.coeff(2) * 180.0 / M_PI, 1.0, 0.0, 0.0);

  glTranslated(motion.translation.coeff(0),
               motion.translation.coeff(1),
               motion.translation.coeff(2));

  glRotated(motion.euler_zyx.coeff(0) * 180.0 / M_PI, 0.0, 0.0, 1.0);
  glRotated(motion.euler_zyx.coeff(1) * 180.0 / M_PI, 0.0, 1.0, 0.0);
  glRotated(motion.euler_zyx.coeff(2) * 180.0 / M_PI, 1.0, 0.0, 0.0);

  glCallList(model_list);
}

bool Object::haveEmptyParam()
{
  if(vertex.size() == 0 || v_idx.size() == 0 || normal.size() == 0 || n_idx.size() == 0 ||
     material.size() == 0 || material_assignment.size() == 0)
  {
    return true;
  }

  return false;
}
