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

#include "ahl_utils/yaml_loader.hpp"

using namespace ahl_utils;

YAMLLoader::YAMLLoader(const std::string& yaml)
{
  ifs_.open(yaml.c_str());
  if(ifs_.fail())
  {
    std::stringstream msg;
    msg << "Could not open \"" << yaml << "\".";
    throw ahl_utils::Exception("YAMLLoader::YAMLLoader", msg.str());
  }

  doc_ = YAML::Load(ifs_);
}

bool YAMLLoader::loadVector(const std::string& tag, Eigen::MatrixXd& dst)
{
  if(!doc_[tag])
    return false;

  try
  {
    dst.resize(doc_[tag].size(), 1);
    for(unsigned int i = 0; i < doc_[tag].size(); ++i)
    {
      dst.coeffRef(i, 0) = doc_[tag][i].as<double>();
    }
  }
  catch(YAML::Exception& e)
  {
    throw ahl_utils::Exception("YAMLLoader::loadVector", e.what());
  }

  return true;
}

bool YAMLLoader::loadMatrix(const std::string& tag, Eigen::MatrixXd& dst)
{
  if(!doc_[tag])
    return false;

  try
  {
    unsigned int rows = doc_[tag].size();
    if(rows == 0)
    {
      throw ahl_utils::Exception("YAMLLoader::loadMatrix", "rows is zero.");
    }
    unsigned int cols = doc_[tag][0].size();

    dst.resize(rows, cols);

    for(unsigned int i = 0; i < rows; ++i)
    {
      for(unsigned int j = 0; j < cols; ++j)
      {
        dst.coeffRef(i, j) = doc_[tag][i][j].as<double>();
      }
    }
  }
  catch(YAML::Exception& e)
  {
    throw ahl_utils::Exception("YAMLLoader::loadVector", e.what());
  }

  return true;
}
