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

#ifndef __GL_WRAPPER_X_OBJECT_HPP
#define __GL_WRAPPER_X_OBJECT_HPP

#include <string>
#include <fstream>
#include <map>
#include <boost/shared_ptr.hpp>
#include <gl_wrapper/object/object.hpp>

namespace gl_wrapper
{

  class XObject
  {
  public:
    XObject(const std::string& name);
    ~XObject();
    void setTranslation(const std::string& key, double x, double y, double z);
    void setEulerZYX(const std::string& key, double rz, double ry, double rx);
    void display();

  private:
    XObject();
    XObject(const XObject& obj);
    XObject& operator=(const XObject& obj);

    void getNextLineOf(const std::string& str, std::string& dst);

    void loadMapKey();
    void loadTransformMatrix(const ObjectPtr& obj, const std::string& key);
    void loadVertex(const ObjectPtr& obj, const std::string& key);
    void loadVertexIndex(const ObjectPtr& obj, const std::string& key);
    void loadNormal(const ObjectPtr& obj, const std::string& key);
    void loadNormalIndex(const ObjectPtr& obj, const std::string& key);
    void loadMaterial(const ObjectPtr& obj, const std::string& key);
    void createDisplayList(const ObjectPtr& obj, const std::string& key);
    void calcOffset(const Eigen::Matrix4d& transform, Object::Offset& offset);

    void printMapKey();
    void printf(const Eigen::Matrix4d& mat);
    void printf(const Eigen::Vector3d& vec);

    Eigen::Matrix4d root_transform_;
    std::ifstream ifs_;
    std::vector<std::string> key_;
    std::map<std::string, ObjectPtr> object_;
  };

  typedef boost::shared_ptr<XObject> XObjectPtr;
}

#endif /* __GL_WRAPPER_X_OBJECT_HPP */
