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


#ifndef __AHL_UTILS_IO_UTILS_HPP
#define __AHL_UTILS_IO_UTILS_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "ahl_utils/str_utils.hpp"

namespace ahl_utils
{

  class IOUtils
  {
  public:
    static void print(const Eigen::MatrixXd& m);

    template<class T>
    static void print(const std::vector<T>& v)
    {
      std::cout << "[ ";
      for(unsigned int i = 0; i < v.size(); ++i)
      {
        std::cout << v[i];
        if(i < v.size() - 1)
        {
          std::cout << ", ";
        }
      }
      std::cout << " ]" << std::endl;
    }

    template<class T>
    static void print(const std::vector< std::vector<T> >& m)
    {
      std::cout << "[ ";
      for(unsigned int i = 0; i < m.size(); ++i)
      {
        for(unsigned int j = 0; j < m[i].size(); ++j)
        {
          std::cout << m[i][j];
          if(j < m[i].size() - 1)
            std::cout << ", ";
        }
        if(i < m.size() - 1)
          std::cout << std::endl;
      }
      std::cout << " ]" << std::endl;
    }

    static bool getValues(std::ifstream& ifs, Eigen::MatrixXd& dst);
    static bool getValues(std::ifstream& ifs, std::vector<Eigen::MatrixXd>& dst);

    template<class T>
    static bool getValues(std::ifstream& ifs, std::vector<T>& dst)
    {
      if(!ifs)
        return false;

      std::string str;
      if(!std::getline(ifs, str))
        return false;

      return StrUtils::convertToVector(str, dst, ",;: \t");
    }

    template<class T>
    static bool getValues(std::ifstream& ifs, std::vector< std::vector<T> >& dst)
    {
      dst.clear();

      std::vector<T> tmp;
      while(IOUtils::getValues(ifs, tmp))
      {
        dst.push_back(tmp);
      }

      if(dst.size() == 0)
        return false;

      return true;
    }

    static unsigned long getLineNum(std::ifstream& ifs);



  private:

  };

}

#endif /* __AHl_UTILS_IO_UTILS_HPP */
