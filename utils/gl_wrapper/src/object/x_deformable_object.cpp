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
#include <string.h>
#include <sstream>
#include <gl_wrapper/object/x_deformable_object.hpp>

using namespace gl_wrapper;

XFileAnalyzer::XFileAnalyzer(const std::string& config_file)
  : separator(": ,"), open_file_(false), get_file_name_(false)
{
  ifs_.open(config_file.c_str());
  if(!ifs_)
  {
    std::cerr << "XDeformableObject::init() : Could not open " << config_file.c_str() << "." << std::endl;
    open_file_ = false;
  }
  else
  {
    open_file_ = true;
  }
}

bool XFileAnalyzer::getFileName(std::string& name)
{
  if(open_file_ == false)
  {
    return false;
  }
  get_file_name_ = true;

  std::string buf;
  while(std::getline(ifs_, buf))
  {
    boost::tokenizer< boost::char_separator<char> > token(buf, separator);
    boost::tokenizer< boost::char_separator<char> >::iterator it = token.begin();

    if(it != token.end() && *it == "file")
    {
      ++it;
      name = *it;
      return true;
    }
  }

  return false;
}

bool XFileAnalyzer::getBoneRelation(std::string& child, std::string& parent)
{
  if(get_file_name_ == false)
  {
    return false;
  }

  std::string buf;
  while(std::getline(ifs_, buf))
  {
    boost::tokenizer< boost::char_separator<char> > token(buf, separator);
    boost::tokenizer< boost::char_separator<char> >::iterator it = token.begin();

    if(it != token.end() && *it == "child")
    {
      ++it;
      if(it != token.end() && *it == "parent")
      {
        ++it;
        child = *it;

        if(it != token.end())
        {
          ++it;
          parent = *it;

          return true;
        }
      }
    }
  }

  return false;
}

void Bone::getBoneName(std::string& buf)
{
  int start_idx = 0;
  for(unsigned int i = 0; i < buf.size(); ++i)
  {
    ++start_idx;
    if(buf[i] == '\"')
    {
      break;
    }
  }

  int end_idx = start_idx + 1;
  for(unsigned int i = end_idx; i < buf.size(); ++i)
  {
    ++end_idx;
    if(buf[i] == '\"')
    {
      break;
    }
  }

  int size = end_idx - start_idx - 1;
  name_.resize(size);
  for(int i = 0; i < size; ++i)
  {
    name_[i] = buf[start_idx + i];
  }
  
}

void XDeformableObject::init(const std::string& config_file)
{
  XFileAnalyzer analyzer(config_file);
  analyzer.getFileName(file_name_);
  setFile(file_name_);
  load();

  std::string child, parent;
  while(analyzer.getBoneRelation(child, parent))
  {
    setParent(child, parent);
  }

  connectBones();
}

void XDeformableObject::setFile(const std::string& file_name)
{
  file_name_ = file_name;
}

void XDeformableObject::load()
{
  //std::cout << "Open " << file_name_ << std::endl;
  ifs_.open(file_name_.c_str());
  if(!ifs_)
  {
    std::cerr << "Error : XLoader::load() : Could not open "
              << file_name_
              << std::endl;
    return;
  }

  loadVertices();
  loadFaces();
  loadMaterials();
  loadNormals();
  loadWeights();
  createDisplayList();

  ifs_.close();
}

//XObject////////////////////////////////////////////////////////
//private////////////////////////////////////////////////////////
void XDeformableObject::skipLine()
{
  std::string str;
  std::getline(ifs_, str);
}

bool XDeformableObject::skipLines(const std::string& str)
{
  std::string buf;
  for(;;)
  {
    if(std::getline(ifs_, buf))
    {
      if(strstr(buf.c_str(), str.c_str()) != NULL)
      {
        return true;
      }
    }
    else
    {
      return false;
    }
  }
}

bool XDeformableObject::skipLines(const std::string& str1, const std::string& str2)
{
  std::string buf;
  for(;;)
  {
    if(std::getline(ifs_, buf))
    {
      if(strstr(buf.c_str(), str1.c_str()) != NULL
         && strstr(buf.c_str(), str2.c_str()) != NULL)
      {
        return true;
      }
    }
    else
    {
      return false;
    }
  }
}

void XDeformableObject::loadVertices()
{
  std::string buf;
  std::istringstream iss;

  this->skipLines("Mesh {");

  std::getline(ifs_, buf);
  iss.str(buf);

  iss >> mesh_.vertex_num_;
  //std::cout << "mesh vertex num : " << mesh_.vertex_num_ << std::endl;

  mesh_.vertex_.resize(mesh_.vertex_num_);
  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    mesh_.vertex_[i].q_.resize(4);

    std::getline(ifs_, buf);

    char colon;
    iss.str(buf);
    iss >> mesh_.vertex_[i].q_[0] >> colon
        >> mesh_.vertex_[i].q_[1] >> colon
        >> mesh_.vertex_[i].q_[2];
    mesh_.vertex_[i].q_[3] = 1.0;
/*
    std::cout << mesh_.vertex_[i].q_[0] << colon
              << mesh_.vertex_[i].q_[1] << colon
              << mesh_.vertex_[i].q_[2] << colon
              << mesh_.vertex_[i].q_[3] << colon << "," << std::endl;
*/
  }
}

void XDeformableObject::loadFaces()
{
  std::string buf;
  std::istringstream iss;

  std::getline(ifs_, buf);
  iss.str(buf);
  iss >> mesh_.face_num_;
  //std::cout << "face num : " << mesh_.face_num_ << std::endl;

  mesh_.used_vertex_num_.resize(mesh_.face_num_);
  mesh_.vertex_order_.resize(mesh_.face_num_);  

  for(int i = 0; i < mesh_.face_num_; ++i)
  {
    std::getline(ifs_, buf);
    iss.str(buf);

    int used_vertex_num;
    char delimiter;

    iss >> mesh_.used_vertex_num_[i];
    mesh_.vertex_order_[i].resize(mesh_.used_vertex_num_[i]);

    iss.str(buf);
    if(mesh_.used_vertex_num_[i] == 3)
    {
      iss >> used_vertex_num >> delimiter
          >> mesh_.vertex_order_[i][0] >> delimiter
          >> mesh_.vertex_order_[i][1] >> delimiter
          >> mesh_.vertex_order_[i][2];
    }
    else if(mesh_.used_vertex_num_[i] == 4)
    {
      iss >> used_vertex_num >> delimiter
          >> mesh_.vertex_order_[i][0] >> delimiter
          >> mesh_.vertex_order_[i][1] >> delimiter
          >> mesh_.vertex_order_[i][2] >> delimiter
          >> mesh_.vertex_order_[i][3];
    }
/*
    std::cout << mesh_.used_vertex_num_[i] << "; ";
    for(int j = 0; j < mesh_.used_vertex_num_[i]; ++j)
    {
      std::cout << mesh_.vertex_order_[i][j] << ", ";
    }
    std::cout << std::endl;
*/
  }
}

void XDeformableObject::loadMaterials()
{
  this->skipLines("MeshMaterialList {");

  mesh_.material_list_.resize(mesh_.face_num_);

  std::string buf;
  std::istringstream iss;

  std::getline(ifs_, buf);
  iss.str(buf);
  iss >> mesh_.material_num_;

  if(mesh_.material_num_ == 0)
  {
    std::cout << file_name_ << " has no materials." << std::endl;
    std::cout << "Default material parameter is used." << std::endl;
    mesh_.material_exist_ = false;
  }
  else
  {
    mesh_.material_exist_ = true;
    //std::cout << file_name_ << " has " << mesh_.material_num_
    //          << " materials." << std::endl;
  }

  this->skipLine();

  for(int i = 0; i < mesh_.face_num_; ++i)
  {
    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.material_list_[i];
    //std::cout << mesh_.material_list_[i] << std::endl;
  }

  mesh_.diffuse_.resize(mesh_.material_num_);
  mesh_.specular_.resize(mesh_.material_num_);
  mesh_.temp_material_info1_.resize(mesh_.material_num_);
  mesh_.temp_material_info2_.resize(mesh_.material_num_);

  const int rgba_channel_num = 4;
  const int rgb_channel_num  = 3;
  char delimiter;
  for(int i = 0; i < mesh_.material_num_; ++i)
  {
    //std::cout << "Material " << i << " {" << std::endl;

    mesh_.diffuse_[i].resize(rgba_channel_num);
    mesh_.specular_[i].resize(rgb_channel_num);
    mesh_.temp_material_info2_[i].resize(rgb_channel_num);

    this->skipLines("Material", "{");
    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.diffuse_[i][0] >> delimiter
        >> mesh_.diffuse_[i][1] >> delimiter
        >> mesh_.diffuse_[i][2] >> delimiter
        >> mesh_.diffuse_[i][3];

    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.temp_material_info1_[i];

    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.specular_[i][0] >> delimiter
        >> mesh_.specular_[i][1] >> delimiter
        >> mesh_.specular_[i][2];

    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.temp_material_info2_[i][0] >> delimiter
        >> mesh_.temp_material_info2_[i][1] >> delimiter
        >> mesh_.temp_material_info2_[i][2];

/*
    for(int j = 0; j < 4; ++j)
    {
      std::cout << mesh_.diffuse_[i][j] << "; ";
    }
    std::cout << std::endl;
    std::cout << mesh_.temp_material_info1_[i] << std::endl;
    for(int j = 0; j < 3; ++j)
    {
      std::cout << mesh_.specular_[i][j] << "; ";
    }
    std::cout << std::endl;
    for(int j = 0; j < 3; ++j)
    {
      std::cout << mesh_.temp_material_info2_[i][j] << "; ";
    }
    std::cout << std::endl << "}" << std::endl;;
*/
  }
}

void XDeformableObject::loadNormals()
{
  std::string buf;
  std::istringstream iss;
  char delimiter;

  this->skipLines("MeshNormals", "{");
  this->skipLine();
  //std::cout << "MeshNormals {" << std::endl;


  mesh_.normal_.resize(mesh_.vertex_num_);
  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    mesh_.normal_[i].resize(4);

    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.normal_[i][0] >> delimiter
        >> mesh_.normal_[i][1] >> delimiter
        >> mesh_.normal_[i][2];

    mesh_.normal_[i][3] = 1.0;

/*
    for(int j = 0; j < 4; ++j)
    {
      std::cout << mesh_.normal_[i][j] << "; ";
    }
    std::cout << std::endl;
*/
  }
  //std::cout << "}" << std::endl;
}

void XDeformableObject::loadWeights()
{
  std::string buf;
  std::istringstream iss;

  if(this->skipLines("XSkinMeshHeader", "{"))
  {
    this->skipLine();
    this->skipLine();

    std::getline(ifs_, buf);
    iss.str(buf);
    iss >> mesh_.bone_num_;

    //std::cout << file_name_ << " has "
    //          << mesh_.bone_num_ << std::endl;
    mesh_.bone_.resize(mesh_.bone_num_);

    for(int i = 0; i < mesh_.vertex_num_; ++i)
    {
      mesh_.vertex_[i].weight_num_ = 0;
      mesh_.vertex_[i].weight_.resize(mesh_.bone_num_);

      for(int j = 0; j < mesh_.bone_num_; ++j)
      {
        mesh_.vertex_[i].weight_[j] = 1.0;
      }
    }

    for(int i = 0; i < mesh_.bone_num_; ++i)
    {
      mesh_.bone_[i].rot_ << 1.0, 0.0, 0.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             0.0, 0.0, 1.0, 0.0,
                             0.0, 0.0, 0.0, 1.0;

      mesh_.bone_[i].level_ = 0;
      this->skipLines("SkinWeights", "{");
      std::getline(ifs_, buf);
      mesh_.bone_[i].getBoneName(buf);

      //std::cout << "bone[" << i << "] name : "
      //          << mesh_.bone_[i].name_ << std::endl;

      std::getline(ifs_, buf);
      iss.str(buf);
      iss >> mesh_.bone_[i].weight_num_;
/*      std::cout << "mesh.bone[" << i << "] "
                << "weight num : " << mesh_.bone_[i].weight_num_
                << std::endl;
*/
      mesh_.bone_[i].weight_index_.resize(mesh_.bone_[i].weight_num_);
      mesh_.bone_[i].weight_.resize(mesh_.bone_[i].weight_num_);

      for(int j = 0; j < mesh_.bone_[i].weight_num_; ++j)
      {
        std::getline(ifs_, buf);
        iss.str(buf);
        iss >> mesh_.bone_[i].weight_index_[j];

      }     

      for(int j = 0; j < mesh_.bone_[i].weight_num_; ++j)
      {
        std::getline(ifs_, buf);
        iss.str(buf);
        iss >> mesh_.bone_[i].weight_[j];
      }

      for(int j = 0; j < mesh_.bone_[i].weight_num_; ++j)
      {
        ++mesh_.vertex_[ mesh_.bone_[i].weight_index_[j] ].weight_num_;
      }

      //各ボーンの根元の位置を取得
      Eigen::Matrix3d trans;
      char delimiter;

      for(int j = 0; j < 3; ++j)
      {
        std::getline(ifs_, buf);
        iss.str(buf);
        iss >> trans.coeffRef(j, 0) >> delimiter
            >> trans.coeffRef(j, 1) >> delimiter
            >> trans.coeffRef(j, 2);
      }

      std::getline(ifs_, buf);
      iss.str(buf);
      iss >> mesh_.bone_[i].q_[0] >> delimiter
          >> mesh_.bone_[i].q_[1] >> delimiter
          >> mesh_.bone_[i].q_[2];

      mesh_.bone_[i].q_ = trans*mesh_.bone_[i].q_*(-1.0);
      mesh_.bone_[i].offset_ = mesh_.bone_[i].q_;

      Eigen::Matrix4d base_rot;
      base_rot << trans.coeff(0, 0), trans.coeff(0, 1), trans.coeff(0, 2), 0.0,
                  trans.coeff(1, 0), trans.coeff(1, 1), trans.coeff(1, 2), 0.0,
                  trans.coeff(2, 0), trans.coeff(2, 1), trans.coeff(2, 2), 0.0,
                  0.0, 0.0, 0.0, 1.0;

      mesh_.bone_[i].base_rot_ = base_rot.transpose();
      //mesh_.bone_[i].base_rot_.show();
      //mesh_.bone_[i].q_.show();
    }

    for(int i = 0; i < mesh_.vertex_num_; ++i)
    {
      mesh_.vertex_[i].weight_bone_index_.resize(mesh_.vertex_[i].weight_num_); 
      int set_index = 0;

      for(int j = 0; j < mesh_.bone_num_; ++j)
      {
        for(int k = 0; k < mesh_.bone_[j].weight_num_; ++k)
        {
          if(i == mesh_.bone_[j].weight_index_[k])
          {
            mesh_.vertex_[i].weight_bone_index_[set_index] = j;
            ++set_index;

            mesh_.vertex_[i].weight_[j] = mesh_.bone_[j].weight_[k];
          }
        }
      }

    }
  }
  else
  {
    std::cerr << file_name_ << " has no bone." << std::endl;
  }
}

void XDeformableObject::createDisplayList()
{
  float dif[4];
  float spe[4];
  float amb[4];
  float shi;
  float emi[4];

  model_list_ = glGenLists(1);
  glNewList(model_list_, GL_COMPILE);

  for(int i = 0; i < mesh_.face_num_; ++i)
  {
    glBegin(GL_TRIANGLE_FAN);

    if(mesh_.material_exist_)
    {
      //0-3:diffuse(RGBA), 4: 5-7:specular 8-10:
      for(int j = 0; j < 4; ++j)
      {
        dif[j] = mesh_.diffuse_[ mesh_.material_list_[i] ][j];
      }
      for(int j = 0; j < 3; ++j)
      {
        spe[j] = mesh_.specular_[ mesh_.material_list_[i] ][j];
        emi[j] = spe[j];
      }
    }

    glMaterialfv(GL_FRONT, GL_DIFFUSE,  dif);
    glMaterialfv(GL_FRONT, GL_SPECULAR, spe);
    glMaterialfv(GL_FRONT, GL_EMISSION, emi);

    for(int j = 0; j < mesh_.used_vertex_num_[i]; ++j)
    {
      glNormal3f(mesh_.normal_[ mesh_.vertex_order_[i][j] ][0],
                 mesh_.normal_[ mesh_.vertex_order_[i][j] ][1],
                 mesh_.normal_[ mesh_.vertex_order_[i][j] ][2]);

      glVertex3d(mesh_.vertex_[ mesh_.vertex_order_[i][j] ].q_[0],
                 mesh_.vertex_[ mesh_.vertex_order_[i][j] ].q_[1],
                 mesh_.vertex_[ mesh_.vertex_order_[i][j] ].q_[2]);
    }

    glEnd();
  }
  glEndList();
}

void XDeformableObject::draw()
{
  glCallList(model_list_);
}

int XDeformableObject::transName2Index(const std::string& bone_name)
{
  for(int i = 0; i < mesh_.bone_num_; ++i)
  {
    if(bone_name == mesh_.bone_[i].name_)
    {
      return i;
    }
  }

  std::cerr << "xLoader::transName2Index error!" << std::endl;
  std::cerr << "Could not find such a bone!" << std::endl;

  return -1;
}

void XDeformableObject::setParent(const std::string& child_bone_name, const std::string& parent_bone_name)
{
  int child_index = transName2Index(child_bone_name);

  //ルートボーンでなければ...
  if(parent_bone_name != "ROOT")
  {
    int parent_index = transName2Index(parent_bone_name);

    mesh_.bone_[child_index].offset_ -= mesh_.bone_[parent_index].q_;
    mesh_.bone_[child_index].parent_ = parent_index;
  }
  else
  {
    mesh_.bone_[child_index].parent_ = -1;
  }
}

void XDeformableObject::connectBones()
{
  //各ボーンがいくつの親ボーンを直列に持つか
  for(int i = 0; i < mesh_.bone_num_; ++i)
  {
    for(int j = i; mesh_.bone_[j].parent_ >= 0; j = mesh_.bone_[j].parent_)
    {
      if(mesh_.bone_[i].parent_ >= 0)
      {
        ++mesh_.bone_[i].level_;
      }
      else
      {
        mesh_.bone_[i].level_ = 0;
      }
    }
  }

  //各頂点に関連するbornのうち、最も直列に親ボーンを持つ数が
  //多いものがどれかを求める
  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    int wbi = mesh_.vertex_[i].weight_bone_index_[0];
    int max_level = mesh_.bone_[wbi].level_;

    mesh_.vertex_[i].youngest_bone_index_ = wbi;

    for(int j = 0; j < mesh_.vertex_[i].weight_num_; ++j)
    {
      wbi = mesh_.vertex_[i].weight_bone_index_[j];

      if(max_level < mesh_.bone_[wbi].level_)
      {
        max_level = mesh_.bone_[wbi].level_;
        mesh_.vertex_[i].youngest_bone_index_ = wbi;
      }
    }
  }

  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    int wbi = mesh_.vertex_[i].youngest_bone_index_;
    int min_level = mesh_.bone_[wbi].level_;

    mesh_.vertex_[i].eldest_bone_index_ = wbi;

    for(int j = 0; j < mesh_.vertex_[i].weight_num_; ++j)
    {
      wbi = mesh_.vertex_[i].weight_bone_index_[j];

      if(min_level > mesh_.bone_[wbi].level_)
      {
        min_level = mesh_.bone_[wbi].level_;
        mesh_.vertex_[i].eldest_bone_index_ = wbi;
      }
    }
  }
}

void XDeformableObject::rotate(const std::string& bone_name, Eigen::Matrix3d& mat)
{
  int bone_index = transName2Index(bone_name);
  Eigen::Matrix4d rot;
  rot << mat.coeff(0, 0), mat.coeff(0, 1), mat.coeff(0, 2), 0.0,
         mat.coeff(1, 0), mat.coeff(1, 1), mat.coeff(1, 2), 0.0,
         mat.coeff(2, 0), mat.coeff(2, 1), mat.coeff(2, 2), 0.0,
         0.0, 0.0, 0.0, 1.0;

  mesh_.bone_[bone_index].rot_ = rot;
}

void XDeformableObject::display()
{
  static float dif[4] = {0.8, 0.8, 0.0, 1.0};
  static float spe[4] = {0.1, 0.1, 0.1, 1.0};
  static float amb[4] = {0.1, 0.1, 0.1, 1.0};
  static float shi[1] = {32.0};
  static float emi[4] = {1.0, 1.0, 1.0, 1.0};

  std::vector<Eigen::Vector4d> v(mesh_.vertex_num_);
  std::vector<Eigen::Vector4d> n(mesh_.vertex_num_);

  static Eigen::Vector4d tempv;
  static Eigen::Vector4d tempn;

  std::vector<Eigen::Matrix4d> T(mesh_.bone_num_);

  for(int i = 0; i < mesh_.bone_num_; ++i)
  {
    T[i] << 1.0, 0.0, 0.0, -mesh_.bone_[i].q_.coeff(0),
            0.0, 1.0, 0.0, -mesh_.bone_[i].q_.coeff(1),
            0.0, 0.0, 1.0, -mesh_.bone_[i].q_.coeff(2),
            0.0, 0.0, 0.0, 1.0;

    int bi = i;
    for(int j = 0; j < mesh_.bone_[i].level_; ++j)
    {
      Eigen::Matrix4d trans;
      trans << 1.0, 0.0, 0.0, mesh_.bone_[bi].offset_.coeff(0),
               0.0, 1.0, 0.0, mesh_.bone_[bi].offset_.coeff(1),
               0.0, 0.0, 1.0, mesh_.bone_[bi].offset_.coeff(2),
               0.0, 0.0, 0.0, 1.0;

      T[i] = trans
             * mesh_.bone_[bi].base_rot_.transpose()
             * mesh_.bone_[bi].rot_
             * mesh_.bone_[bi].base_rot_
             * T[i];

      bi = mesh_.bone_[bi].parent_;
    }
  }

  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    for(int j = 0; j < mesh_.vertex_[i].weight_num_; ++j)
    {
      int wbi = mesh_.vertex_[i].weight_bone_index_[j];

      getTrans(T[wbi], mesh_.vertex_[i].q_, tempv);
      getTrans(T[wbi], mesh_.normal_[i], tempn); 

      v[i] += tempv * mesh_.vertex_[i].weight_[wbi];
      n[i] += tempn * mesh_.vertex_[i].weight_[wbi];
    }

    n[i] = n[i].normalized();
  }

  for(int i = 0; i < mesh_.face_num_; ++i)
  {
    glBegin(GL_TRIANGLE_FAN);

    if(mesh_.material_exist_)
    {
      for(int j = 0; j < 4; ++j)
      {
        dif[j] = mesh_.diffuse_[ mesh_.material_list_[i] ][j];
      }
      for(int j = 0; j < 3; ++j)
      {
        spe[j] = mesh_.specular_[ mesh_.material_list_[i] ][j];
        emi[j] = spe[j];
      }
    }

    glMaterialfv(GL_FRONT, GL_DIFFUSE,   dif);
    glMaterialfv(GL_FRONT, GL_SPECULAR,  spe);
    glMaterialfv(GL_FRONT, GL_EMISSION,  emi);

    for(int j = 0; j < mesh_.used_vertex_num_[i]; ++j)
    {
      glNormal3f(n[ mesh_.vertex_order_[i][j] ].coeff(0), 
                 n[ mesh_.vertex_order_[i][j] ].coeff(1), 
                 n[ mesh_.vertex_order_[i][j] ].coeff(2));
      glVertex3d(v[ mesh_.vertex_order_[i][j] ].coeff(0), 
                 v[ mesh_.vertex_order_[i][j] ].coeff(1), 
                 v[ mesh_.vertex_order_[i][j] ].coeff(2));
    }
    glEnd();
  }
}

void XDeformableObject::displayWithoutShade()
{
  static float dif[4] = {0.8, 0.8, 0.0, 1.0};
  static float spe[4] = {0.1, 0.1, 0.1, 1.0};
  static float amb[4] = {0.1, 0.1, 0.1, 1.0};
  static float shi[1] = {32.0};
  static float emi[4] = {1.0, 1.0, 1.0, 1.0};

  //manipit::math::Vector<4, double> v[mesh_.vertex_num_];
  std::vector<Eigen::Vector4d> v(mesh_.vertex_num_);
  //manipit::math::Vector<4, double> n[mesh_.vertex_num_];
  std::vector<Eigen::Vector4d> n(mesh_.vertex_num_);

  static Eigen::Vector4d tempv;
  static Eigen::Vector4d tempn;

  std::vector<Eigen::Matrix4d> T(mesh_.bone_num_);

  for(int i = 0; i < mesh_.bone_num_; ++i)
  {
    T[i] << 1.0, 0.0, 0.0, -mesh_.bone_[i].q_.coeff(0),
            0.0, 1.0, 0.0, -mesh_.bone_[i].q_.coeff(1),
            0.0, 0.0, 1.0, -mesh_.bone_[i].q_.coeff(2),
            0.0, 0.0, 0.0, 1.0;

    int bi = i;
    for(int j = 0; j < mesh_.bone_[i].level_; ++j)
    {
      Eigen::Matrix4d trans;
      trans << 1.0, 0.0, 0.0, mesh_.bone_[bi].offset_.coeff(0),
               0.0, 1.0, 0.0, mesh_.bone_[bi].offset_.coeff(1),
               0.0, 0.0, 1.0, mesh_.bone_[bi].offset_.coeff(2),
               0.0, 0.0, 0.0, 1.0;

      T[i] = trans
             * mesh_.bone_[bi].base_rot_.transpose()
             * mesh_.bone_[bi].rot_
             * mesh_.bone_[bi].base_rot_
             * T[i];

      bi = mesh_.bone_[bi].parent_;
    }
  }

  for(int i = 0; i < mesh_.vertex_num_; ++i)
  {
    for(int j = 0; j < mesh_.vertex_[i].weight_num_; ++j)
    {
      int wbi = mesh_.vertex_[i].weight_bone_index_[j];

      getTrans(T[wbi], mesh_.vertex_[i].q_, tempv);
      getTrans(T[wbi], mesh_.normal_[i], tempn); 

      v[i] += tempv*mesh_.vertex_[i].weight_[wbi];
      n[i] += tempn*mesh_.vertex_[i].weight_[wbi];
    }

    n[i] /= n[i].norm();
  }

  for(int i = 0; i < mesh_.face_num_; ++i)
  {
    glBegin(GL_TRIANGLE_FAN);

    if(mesh_.material_exist_)
    {
      for(int j = 0; j < 4; ++j)
      {
        dif[j] = mesh_.diffuse_[ mesh_.material_list_[i] ][j];
      }
      for(int j = 0; j < 3; ++j)
      {
        spe[j] = mesh_.specular_[ mesh_.material_list_[i] ][j];
        emi[j] = spe[j];
      }
    }

    //glMaterialfv(GL_FRONT, GL_DIFFUSE,   dif);
    //glMaterialfv(GL_FRONT, GL_SPECULAR,  spe);
    //glMaterialfv(GL_FRONT, GL_EMISSION,  emi);
    glColor3f(dif[0], dif[1], dif[2]);

    for(int j = 0; j < mesh_.used_vertex_num_[i]; ++j)
    {
      glNormal3f(n[ mesh_.vertex_order_[i][j] ].coeff(0), 
                 n[ mesh_.vertex_order_[i][j] ].coeff(1), 
                 n[ mesh_.vertex_order_[i][j] ].coeff(2));
      glVertex3d(v[ mesh_.vertex_order_[i][j] ].coeff(0), 
                 v[ mesh_.vertex_order_[i][j] ].coeff(1), 
                 v[ mesh_.vertex_order_[i][j] ].coeff(2));
    }

    glEnd();
  }
}

void XDeformableObject::getTrans(Eigen::Matrix4d& src_mat,
                                 std::vector<double>& src_vec,
                                 Eigen::Vector4d& dst_vec)
{
  Eigen::Vector4d temp = Eigen::Vector4d::Zero();

  for(unsigned int i = 0; i < 3; ++i)
  {
    for(unsigned int j = 0; j < 4; ++j)
    {
      temp.coeffRef(i) += src_mat.coeff(i, j) * src_vec[j];
    }
  }

  dst_vec = temp;
}
