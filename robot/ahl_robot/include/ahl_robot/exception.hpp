/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
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

//////////////////////////////////////////////
/// \file exception.hpp
/// \brief Declare ahl_robot::Exception class
/// \author Daichi Yoshikawa
//////////////////////////////////////////////

#ifndef __AHL_ROBOT_EXCEPTION_HPP
#define __AHL_ROBOT_EXCEPTION_HPP

#include <sstream>

namespace ahl_robot
{

  /// ahl_robot::Exception
  class Exception
  {
  public:
    /// Constructor
    /// \param src Name of function which threw exception
    /// \param msg Description of exception
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg) {}

    /// Get description of exception
    /// \return description of exception
    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "ahl_robot::Exception occuered." << std::endl
                << "  src : " << src_ << std::endl
                << "  msg : " << msg_;

      return ss.str().c_str(); 
    }

  private:
    //! Name of function which threw exception
    std::string src_;
    //! Description of exception
    std::string msg_;
  };

}

#endif /* __AHL_ROBOT_EXCEPTION_HPP */
