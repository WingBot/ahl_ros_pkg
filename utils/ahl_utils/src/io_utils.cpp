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

#include "ahl_utils/io_utils.hpp"

using namespace ahl_utils;

void IOUtils::print(const Eigen::MatrixXd& m)
{
  std::cout << "[ ";
  for(unsigned int i = 0; i < m.rows(); ++i)
  {
    for(unsigned int j = 0; j < m.cols(); ++j)
    {
      std::cout << m.coeff(i, j);
      if(j < m.cols() - 1)
        std::cout << ", ";
    }
    if(i < m.rows() - 1)
      std::cout << std::endl;
  }
  std::cout << " ]" << std::endl;
}

bool IOUtils::getValues(std::ifstream& ifs, Eigen::MatrixXd& dst)
{
  if(!ifs)
    return false;

  std::string str;
  if(!std::getline(ifs, str))
    return false;

  std::vector<std::string> words;
  StrUtils::separate(str, words, ",;: \t");

  dst = Eigen::MatrixXd::Zero(words.size(), 1);
  for(unsigned int j = 0; j < words.size(); ++j)
  {
    StrUtils::convertToNum(words[j], dst.coeffRef(j, 0));
  }

  return true;
}

bool IOUtils::getValues(std::ifstream& ifs, std::vector<Eigen::MatrixXd>& dst)
{
  Eigen::MatrixXd temp;
  while(IOUtils::getValues(ifs, temp))
  {
    dst.push_back(temp);
  }

  if(dst.size() == 0)
    return false;

  return true;
}

unsigned long IOUtils::getLineNum(std::ifstream& ifs)
{



}
