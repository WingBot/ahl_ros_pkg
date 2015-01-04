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

#ifndef __NEURAL_NETWORK_EXCEPTIONS_HPP
#define __NEURAL_NETWORK_EXCEPTIONS_HPP

#include <iostream>
#include <cstdlib>
#include <sstream>

namespace nn
{

  class Exception
  {
  public:
    Exception(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg) {}

    std::string what()
    {
      std::stringstream ss;
      ss << "nn::Exception was thrown." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

  class FatalException
  {
  public:
    FatalException(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg) {}

    std::string what()
    {
      std::stringstream ss;
      ss << "nn::FatalException was thrown." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

  class Shutdown
  {
  public:
    Shutdown(const std::string& src, const std::string& msg)
    {
      std::cerr << "nn::Shutdown was called." << std::endl
                << "This means that exception occuered in constructor." << std::endl
                << "  src : " << src << std::endl
                << "  msg : " << msg << std::endl
                << "Shutting down the process ..." << std::endl;
      exit(1);
    }
  };

}

#endif /* __NEURAL_NETWORK_EXCEPTIONS_HPP */
