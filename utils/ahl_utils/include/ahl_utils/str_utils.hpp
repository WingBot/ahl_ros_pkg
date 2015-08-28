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

#ifndef __AHL_UTILS_STR_UTILS_HPP
#define __AHL_UTILS_STR_UTILS_HPP
#include <string>
#include <sstream>
#include <vector>

namespace ahl_utils
{

  class StrUtils
  {
  public:
    static bool have(const std::string& str, char ch);
    static bool have(const std::string& str, const std::vector<char>& chs);
    static bool have(const std::string& str, const std::string& word);
    static bool isAlphabet(const std::string& str);

    static bool convertToBoolean(const std::string& str, bool& dst)
    {
      std::stringstream ss(str);
      std::string tmp;

      if(!(ss >> tmp))
      {
        return false;
      }

      if(tmp == std::string("true") || tmp == std::string("TRUE"))
      {
        dst = true;
      }
      else if(tmp == std::string("false") || tmp == std::string("FALSE"))
      {
        dst = false;
      }
      else
      {
        return false;
      }

      return true;
    }

    template <class T>
    static bool convertToNum(const std::string& str, T& dst)
    {
      std::stringstream ss(str);
      double tmp;

      if(!(ss >> tmp))
      {
        return false;
      }

      dst = static_cast<T>(tmp);
      return true;
    }

    template <class T>
    static bool convertToVector(const std::string& str, std::vector<T>& vec, const std::string& delimiters)
    {
      std::vector<std::string> words;
      StrUtils::separate(str, words, delimiters);

      if(words.size() == 0)
        return false;

      vec.resize(words.size());
      for(unsigned int i = 0; i < words.size(); ++i)
      {
        if(!StrUtils::convertToNum<T>(words[i], vec[i]))
        {
          return false;
        }
      }

      return true;
    }

    static bool isIPAddress(const std::string& str);
    static void removeSpace(std::string& str);
    static void removeIndent(std::string& str);
    static void remove(std::string& str, char ch);
    static void separate(const std::string& str, std::vector<std::string>& words, const std::string& delimiters);
  };

}

#endif /* __AHL_UTILS_STR_UTILS_HPP */
