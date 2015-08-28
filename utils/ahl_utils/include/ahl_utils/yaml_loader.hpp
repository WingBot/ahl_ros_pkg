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

#ifndef __AHL_UTILS_YAML_LOADER_HPP
#define __AHL_UTILS_YAML_LOADER_HPP

#include <string>
#include <fstream>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "ahl_utils/exception.hpp"

namespace ahl_utils
{

  class YAMLLoader
  {
  public:
    YAMLLoader(const std::string& yaml);

    template<class T>
    bool loadValue(const std::string& tag, T& dst)
    {
      if(!doc_[tag])
        return false;

      try
      {
        dst = doc_[tag].as<T>();
      }
      catch(YAML::Exception& e)
      {
        throw ahl_utils::Exception("YAMLLoader::loadValue", e.what());
      }

      return true;
    }

    template<class T>
    bool loadVector(const std::string& tag, std::vector<T>& dst)
    {
      if(!doc_[tag])
        return false;

      try
      {
        dst.resize(doc_[tag].size());
        for(unsigned int i = 0; i < doc_[tag].size(); ++i)
        {
          dst[i] = doc_[tag][i].as<T>();
        }
      }
      catch(YAML::Exception& e)
      {
        throw ahl_utils::Exception("YAMLLoader::loadVector", e.what());
      }

      return true;
    }

    bool loadVector(const std::string& tag, Eigen::MatrixXd& dst);
    bool loadMatrix(const std::string& tag, Eigen::MatrixXd& dst);

  private:
    std::ifstream ifs_;
    YAML::Node doc_;
  };

  typedef boost::shared_ptr<YAMLLoader> YAMLLoaderPtr;
}

#endif /* __AHL_UTILS_YAML_LOADER_HPP */
