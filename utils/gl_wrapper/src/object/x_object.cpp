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

#include <iostream>
#include <gl_wrapper/object/x_object.hpp>
#include <gl_wrapper/exception/exceptions.hpp>
#include <ahl_utils/str_utils.hpp>

using namespace gl_wrapper;
using namespace ahl_utils;

XObject::XObject(const std::string& name)
{
  ifs_.open(name.c_str());
  if(!ifs_)
  {
    std::stringstream msg;
    msg << "Could not open \"" << name << "\".";

    throw Exception("XObject::XObject", msg.str());
  }

  std::cout << "Opened \"" << name << "\" successfully." << std::endl;

  this->loadMapKey();
  this->printMapKey();

  ifs_.clear();
  ifs_.seekg(0, std::ios_base::beg);

  for(unsigned int i = 0; i < key_.size(); ++i)
  {
    ObjectPtr obj = ObjectPtr(new Object());

    std::cout << "Load " << key_[i] << " data -->" << std::endl;

    std::cout << "Load transform matrix ... ";
    this->loadTransformMatrix(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "Load vertex ... ";
    this->loadVertex(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "Load vertex index ... ";
    this->loadVertexIndex(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "Load normal ... ";
    this->loadNormal(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "Load normal index ... ";
    this->loadNormalIndex(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "Load Material ... ";
    this->loadMaterial(obj, key_[i]);
    std::cout << "Success." << std::endl;

    std::cout << "<-- " << key_[i] << " data were loaded successfully." << std::endl;

    this->createDisplayList(obj, key_[i]);
    object_[key_[i]] = obj;
  }
}

XObject::~XObject()
{
  for(unsigned int i = 0; i < key_.size(); ++i)
  {
    if(object_[key_[i]].get() != NULL)
    {
      glDeleteLists(object_[key_[i]]->model_list, 1);
    }
  }
}

void XObject::setTranslation(const std::string& key, double x, double y, double z)
{
  //TODO if map doesn't the key

  object_[key]->motion.translation.coeffRef(0) = x;
  object_[key]->motion.translation.coeffRef(1) = y;
  object_[key]->motion.translation.coeffRef(2) = z;
}

void XObject::setEulerZYX(const std::string& key, double rz, double ry, double rx)
{
  //TODO if map doesn't the key

  object_[key]->motion.euler_zyx.coeffRef(0) = rz;
  object_[key]->motion.euler_zyx.coeffRef(1) = ry;
  object_[key]->motion.euler_zyx.coeffRef(2) = rx;
}

void XObject::display()
{
  for(unsigned int i = 0; i < key_.size(); ++i)
  {
    object_[key_[i]]->display();
  }
}

void XObject::getNextLineOf(const std::string& str, std::string& dst)
{
  while(std::getline(ifs_, dst))
  {
    if(StrUtils::have(dst, str))
    {
      break;
    }
  }

  std::getline(ifs_, dst);
}

void XObject::loadMapKey()
{
  std::string str;

  while(std::getline(ifs_, str))
  {
    if(StrUtils::have(str, "Frame "))
    {
      std::vector<std::string> words;
      StrUtils::separate(str, words, " ");

      std::string key("");
      for(int i = 1; i < words.size() - 1; ++i)
      {
        key += words[i];
      }

      if(key != "Root")
      {
        key_.push_back(key);
      }
    }
  }
}

void XObject::loadTransformMatrix(const ObjectPtr& obj, const std::string& key)
{
  std::string str;

  this->getNextLineOf("Frame " + key, str);

  int col = 0;
  Eigen::Matrix4d transform;
      
  while(std::getline(ifs_, str))
  {
    if(StrUtils::have(str, "}"))
      break;

    std::vector<std::string> words;
    StrUtils::removeSpace(str);
    StrUtils::separate(str, words, ",;");

    if(words.size() != transform.rows())
      continue;

    for(int row = 0; row < transform.rows(); ++row)
    {
      StrUtils::convertToNum(words[row], transform.coeffRef(row, col));
    }
    ++col;
  }

  this->calcOffset(transform, obj->offset);
}

void XObject::loadVertex(const ObjectPtr& obj, const std::string& key)
{
  std::string str;
  this->getNextLineOf("Mesh {", str);

  StrUtils::removeSpace(str);
  double size = 0;
  StrUtils::convertToNum(str, size);

  obj->vertex.resize(static_cast<unsigned int>(size));

  for(unsigned int i = 0; i < obj->vertex.size(); ++i)
  {
    if(!std::getline(ifs_, str))
    {
      std::stringstream msg;
      msg << "Could not read line" << i << ".";
      throw Exception("XObject::loadVertex", msg.str());
    }

    std::vector<std::string> words;
    StrUtils::separate(str, words, ";, \t");

    if(words.size() != obj->vertex[i].size())
    {
      std::stringstream msg;
      msg << "Line" << i << " has " << words.size() << " words and it's invalid.";
      throw Exception("XObject::loadVertex", msg.str());
    }

    for(unsigned int row = 0; row < obj->vertex[i].size(); ++row)
    {
      StrUtils::convertToNum(words[row], obj->vertex[i].coeffRef(row));
    }
  }
}

void XObject::loadVertexIndex(const ObjectPtr& obj, const std::string& key)
{
  std::string str;
  std::getline(ifs_, str);
  StrUtils::removeSpace(str);

  double size = 0.0;
  StrUtils::convertToNum(str, size);

  obj->v_idx.resize(static_cast<unsigned int>(size));

  for(unsigned int i = 0; i < obj->v_idx.size(); ++i)
  {
    double v_num = 0.0;
    std::vector<std::string> words;

    std::getline(ifs_, str);
    StrUtils::separate(str, words, ";, \t");
    if(words.size() < 4)
    {
      throw Exception("XObject::loadVertexIndex", "Could not read valid values.");
    }
    StrUtils::convertToNum(words[0], v_num);

    obj->v_idx[i].resize(v_num);
    for(unsigned int j = 0; j < obj->v_idx[i].size(); ++j)
    {
      double idx;
      StrUtils::convertToNum(words[j + 1], idx);
      obj->v_idx[i][j] = static_cast<unsigned int>(idx);
    }
  }
}

void XObject::loadNormal(const ObjectPtr& obj, const std::string& key)
{
  std::string str;
  this->getNextLineOf("MeshNormals {", str);

  double size = 0.0;
  StrUtils::convertToNum(str, size);
  obj->normal.resize(static_cast<unsigned int>(size));

  for(unsigned int i = 0; i < obj->normal.size(); ++i)
  {
    std::getline(ifs_, str);

    std::vector<std::string> words;
    StrUtils::separate(str, words, ";, \t");

    if(words.size() != obj->normal[i].size())
    {
      throw Exception("XObject::loadNormal", "Could not read valid value.");
    }

    for(unsigned row = 0; row < obj->normal[i].size(); ++row)
    {
      StrUtils::convertToNum(words[row], obj->normal[i].coeffRef(row));
    }
  }
}

void XObject::loadNormalIndex(const ObjectPtr& obj, const std::string& key)
{
  std::string str;
  std::getline(ifs_, str);

  double size;
  StrUtils::convertToNum(str, size);
  obj->n_idx.resize(static_cast<unsigned int>(size));

  for(unsigned int i = 0; i < obj->n_idx.size(); ++i)
  {
    double n_num;
    std::getline(ifs_, str);

    std::vector<std::string> words;
    StrUtils::separate(str, words, ";, \t");

    if(words.size() < 4)
    {
      std::stringstream msg;
      msg << "Line has " << words.size() << " words and it's invalid.";
      throw Exception("XObject::loadNormalIndex", msg.str());
    }

    StrUtils::convertToNum(words[0], n_num);
    obj->n_idx[i].resize(static_cast<unsigned int>(n_num));

    for(unsigned int j = 0; j < obj->n_idx[i].size(); ++j)
    {
      double idx;
      StrUtils::convertToNum(words[j + 1], idx);
      obj->n_idx[i][j] = static_cast<unsigned int>(idx);
    }
  }
}

void XObject::loadMaterial(const ObjectPtr& obj, const std::string& key)
{
  std::string str;
  this->getNextLineOf("MeshMaterialList ", str);

  double material_num;
  StrUtils::convertToNum(str, material_num);
  obj->material.resize(static_cast<unsigned int>(material_num));

  std::getline(ifs_, str);
  double face_num;
  StrUtils::convertToNum(str, face_num);
  obj->material_assignment.resize(static_cast<unsigned int>(face_num));

  for(unsigned int i = 0; i < obj->material_assignment.size(); ++i)
  {
    double idx;
    std::getline(ifs_, str);
    StrUtils::convertToNum(str, idx);
    obj->material_assignment[i] = static_cast<unsigned int>(idx);
  }

  GLfloat ambient[4] = {0.f, 0.f, 0.f, 1.f};
  GLfloat diffuse[4] = {0.f, 0.f, 0.f, 1.f};
  GLfloat specular[4] = {0.f, 0.f, 0.f, 1.f};
  GLfloat shine = 127.f;

  double tmp_dif[4];
  double tmp_spe[4];
  double tmp_shi;

  for(int i = 0; i < obj->material.size(); ++i)
  {
    std::vector<std::string> words;

    this->getNextLineOf("Material ", str);
    StrUtils::separate(str, words, ";, \t");
    for(int j = 0; j < words.size(); ++j)
    {
      StrUtils::convertToNum(words[j], tmp_dif[j]);
      diffuse[j] = static_cast<GLfloat>(tmp_dif[j]);
      ambient[j] = static_cast<GLfloat>(tmp_dif[j]);
    }

    std::getline(ifs_, str);
    StrUtils::convertToNum(str, tmp_shi);
    shine = static_cast<GLfloat>(tmp_shi);

    std::getline(ifs_, str);
    StrUtils::separate(str, words, ";, \t");
    for(int j = 0; j < words.size(); ++j)
    {
      StrUtils::convertToNum(words[j], tmp_spe[j]);
      specular[j] = static_cast<GLfloat>(tmp_spe[j]);
    }

    obj->material[i] = MaterialPtr(new Material(ambient, diffuse, specular, shine));
  }
}

void XObject::createDisplayList(const ObjectPtr& obj, const std::string& key)
{
  obj->model_list = glGenLists(1);
  glNewList(obj->model_list, GL_COMPILE);

  unsigned int face_num = obj->v_idx.size();

  for(unsigned int i = 0; i < face_num; ++i)
  {
    glBegin(GL_TRIANGLE_FAN);

    obj->material[obj->material_assignment[i]]->apply();

    for(unsigned int j = 0; j < obj->v_idx[i].size(); ++j)
    {
      glNormal3f(static_cast<GLfloat>(obj->normal[ obj->n_idx[i][j] ].coeff(0)),
                 static_cast<GLfloat>(obj->normal[ obj->n_idx[i][j] ].coeff(1)),
                 static_cast<GLfloat>(obj->normal[ obj->n_idx[i][j] ].coeff(2)));
      glVertex3d(static_cast<GLfloat>(obj->vertex[ obj->v_idx[i][j] ].coeff(0)),
                 static_cast<GLfloat>(obj->vertex[ obj->v_idx[i][j] ].coeff(1)),
                 static_cast<GLfloat>(obj->vertex[ obj->v_idx[i][j] ].coeff(2)));
    }
    glEnd();
  }
  glEndList();
}

void XObject::calcOffset(const Eigen::Matrix4d& transform, Object::Offset& offset)
{
  offset.translation = transform.block(0, 3, 3, 1);

  double r11 = transform.coeff(0, 0);
  double r12 = transform.coeff(0, 1);
  double r13 = transform.coeff(0, 2);
  double r21 = transform.coeff(1, 0);
  double r22 = transform.coeff(1, 1);
  double r23 = transform.coeff(1, 2);
  double r31 = transform.coeff(2, 0);
  double r32 = transform.coeff(2, 1);
  double r33 = transform.coeff(2, 2);

  double cos_beta = std::sqrt(r11 * r11 + r21 * r21);
  if(cos_beta != 0.0)
  {
    offset.euler_zyx.coeffRef(0) = std::atan2(r21, r11);
    offset.euler_zyx.coeffRef(1) = std::atan2(-r31, cos_beta);
    offset.euler_zyx.coeffRef(2) = std::atan2(r32, r33);
  }
  else
  {
    offset.euler_zyx.coeffRef(0) = 0.0;
    offset.euler_zyx.coeffRef(1) = M_PI * 0.5;
    offset.euler_zyx.coeffRef(2) = std::atan2(r12, r22);
  }
}

void XObject::printMapKey()
{
  std::cout << "Loaded map keys are ..." << std::endl;
  for(int i = 0; i < key_.size(); ++i)
  {
    std::cout << "key[" << i << "] : " << key_[i] << std::endl;
  }
}

void XObject::printf(const Eigen::Matrix4d& mat)
{
  for(int i = 0; i < mat.rows(); ++i)
  {
    for(int j = 0; j < mat.cols(); ++j)
    {
      std::cout << mat.coeff(i, j) << " ";
    }
    std::cout << std::endl;
  }
}

void XObject::printf(const Eigen::Vector3d& vec)
{
  for(int i = 0; i < vec.rows(); ++i)
  {
    std::cout << vec.coeff(i) << " ";
  }
  std::cout << std::endl;
}
