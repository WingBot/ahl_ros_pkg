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

#ifndef __CV_WRAPPER_HSV_MASK_PARAM_HPP
#define __CV_WRAPPER_HSV_MASK_PARAM_HPP

#include <boost/shared_ptr.hpp>

namespace cv_wrapper
{

  class HSVMaskParam
  {
  public:
    HSVMaskParam(int h_min, int h_max, int s_min, int s_max, int v_min, int v_max)
      : h_min_(h_min), h_max_(h_max), s_min_(s_min), s_max_(s_max), v_min_(v_min), v_max_(v_max) {}

    const int getHMin() const
    {
      return h_min_;
    }

    const int getHMax() const
    {
      return h_max_;
    }

    const int getSMin() const
    {
      return s_min_;
    }

    const int getSMax() const
    {
      return s_max_;
    }

    const int getVMin() const
    {
      return v_min_;
    }

    const int getVMax() const
    {
      return v_max_;
    }

  private:
    int h_min_;
    int h_max_;
    int s_min_;
    int s_max_;
    int v_min_;
    int v_max_;
  };

  typedef boost::shared_ptr<HSVMaskParam> HSVMaskParamPtr;
}

#endif /* __CV_WRAPPER_HSV_MASK_PARAM_HPP */
