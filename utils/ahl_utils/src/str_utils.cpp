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

#include <cctype>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#include "ahl_utils/str_utils.hpp"

using namespace ahl_utils;

bool StrUtils::have(const std::string& str, char ch)
{
  unsigned int length = str.length();

  for(unsigned int i = 0; i < length; ++i)
  {
    if(str[i] == ch)
      return false;
  }

  return true;
}

bool StrUtils::have(const std::string& str, const std::vector<char>& chs)
{
  unsigned int size = chs.size();

  for(unsigned int i = 0; i < size; ++i)
  {
    if(!StrUtils::have(str, chs[i]))
      return false;
  }

  return true;
}

bool StrUtils::have(const std::string& str, const std::string& word)
{
  return boost::contains(str, word);
}

bool StrUtils::isAlphabet(const std::string& str)
{
  unsigned int length = str.length();

  if(length == 0)
    return false;

  for(unsigned int i = 0; i < length; ++i)
  {
    if(!std::isalpha(str[i]))
      return false;
  }

  return true;
}
/*
bool StrUtils::convertToNum(const std::string& str, double& dst)
{
  std::stringstream ss(str);
  ss >> dst;

  return true;
}
*/
bool StrUtils::isIPAddress(const std::string& str)
{
  std::string temp = str;

  StrUtils::removeSpace(temp);
  std::vector<std::string> words;

  StrUtils::separate(temp, words, ".:");

  if(words.size() != 4)
    return false;

  for(int i = 0; i < words.size(); ++i)
  {
    for(int j = 0; j < words[i].length(); ++j)
    {
      if(words[i][j] < '0' || '9' < words[i][j])
        return false;
    }

    if(std::atoi(words[i].c_str()) < 0 || 255 < std::atoi(words[i].c_str()))
    {
      return false;
    }
  }

  return true;
}

void StrUtils::removeSpace(std::string& str)
{
  size_t pos;
  while((pos = str.find_first_of(" 　\t")) != std::string::npos){
    str.erase(pos, 1);
  }
}

void StrUtils::removeIndent(std::string& str)
{
  for(unsigned int i = 0; i < str.length(); ++i)
  {
    if(str[i] == ' ' || str[i] == '\t')
    {
      str.erase(i, 1);
      --i;
    }
    else
      return;
  }
}

void StrUtils::remove(std::string& str, char ch)
{
  for(unsigned int i = 0; i < str.length(); ++i)
  {
    if(str[i] == ch)
    {
      str.erase(i, 1);
      --i;
    }
  }
}

void StrUtils::separate(const std::string& str, std::vector<std::string>& words, const std::string& delimiters)
{
  typedef boost::char_separator<char> char_separator;
  typedef boost::tokenizer<char_separator> tokenizer;

  char_separator sep(delimiters.c_str(), "", boost::keep_empty_tokens);
  tokenizer tokens(str, sep);

  words.clear();
  for(tokenizer::iterator it = tokens.begin(); it != tokens.end(); ++it)
  {
    if(*it != "")
      words.push_back(*it);
  }
}

