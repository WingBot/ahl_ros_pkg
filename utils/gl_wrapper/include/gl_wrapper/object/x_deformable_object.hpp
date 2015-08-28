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

#ifndef __MANIPIT_GL_WRAPPER_XLOADER_HPP
#define __MANIPIT_GL_WRAPPER_XLOADER_HPP
#include <cmath>
#include <string>
#include <fstream>
#include <vector>
#include <GL/glut.h>
#include <boost/tokenizer.hpp>
#include <Eigen/Dense>
//#include <linear_algebra/vector.hpp>
//#include <linear_algebra/matrix.hpp>

namespace gl_wrapper
{
  class XFileAnalyzer
  {
  public:
    XFileAnalyzer(const std::string& config_file);
    ~XFileAnalyzer() {}

    bool getFileName(std::string& name);
    bool getBoneRelation(std::string& child, std::string& parent);
  private:
    XFileAnalyzer() {}
    XFileAnalyzer(const XFileAnalyzer& analyzer) {}
    void operator=(const XFileAnalyzer& analyzer) {}

    std::ifstream ifs_;
    bool open_file_;
    bool get_file_name_;
    std::string buf;
    boost::char_separator<char> separator;
    boost::tokenizer< boost::char_separator<char> >::iterator it_;
  };

  class Bone
  {
  public:
    void getBoneName(std::string& buf);

    std::string name_;

    //manipit::math::Vector<3, double> q_;
    //manipit::math::Vector<3, double> offset_;
    Eigen::Vector3d q_;
    Eigen::Vector3d offset_;

    //manipit::math::Matrix<4, 4, double> base_rot_;
    //manipit::math::Matrix<4, 4, double> rot_;
    Eigen::Matrix4d base_rot_;
    Eigen::Matrix4d rot_;

    int parent_;
    int weight_num_;

    std::vector<int> weight_index_;
    std::vector<double> weight_;

    int level_;//rootが0
  };

  class Vertex
  {
  public:
    std::vector<double> q_;

    int youngest_bone_index_;
    int eldest_bone_index_;
    int weight_num_;
    std::vector<int> weight_bone_index_;
    std::vector<double> weight_;

    Vertex() : weight_num_(0) {}
    ~Vertex() {}
  };

  class Mesh
  {
  public:
    std::vector<Vertex> vertex_;
    int vertex_num_;
    std::vector< std::vector<int> > vertex_order_;
    std::vector<int> used_vertex_num_;

    int face_num_;

    int material_num_;
    std::vector<int> material_list_;

    std::vector< std::vector<float> > diffuse_;
    std::vector< std::vector<float> > specular_;
    std::vector<float> temp_material_info1_;
    std::vector< std::vector<float> > temp_material_info2_;

    bool material_exist_;

    std::vector< std::vector<double> > normal_;

    std::vector<Bone> bone_;
    int bone_num_;

    Mesh() {}
    ~Mesh() {}
  };

  class XDeformableObject
  {
  public:
    XDeformableObject() {}
    ~XDeformableObject() {}

    void init(const std::string& config_file = "");
    void draw();

    void rotate(const std::string& bone_name, Eigen::Matrix3d& mat);
    void display();
    void displayWithoutShade();

  private:
    void setFile(const std::string& file_name);
    void load();
    void setParent(const std::string& child_bone_name, const std::string& parent_bone_name);
    void connectBones();
    int transName2Index(const std::string& bone_name);

    void skipLine();
    bool skipLines(const std::string& str);
    bool skipLines(const std::string& str1, const std::string& str2);
    void loadVertices();
    void loadFaces();
    void loadMaterials();
    void loadNormals();
    void loadWeights();
    void createDisplayList();
    void getTrans(Eigen::Matrix4d& src_mat, std::vector<double>& src_vec, Eigen::Vector4d& dst_vec);

    std::ifstream ifs_;
    std::string file_name_;

    GLuint model_list_;
    Mesh mesh_;
  };

}
#endif
