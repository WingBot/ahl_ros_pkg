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

#include "train_with_cg/collect_image/hand_pose.hpp"
#include "train_with_cg/collect_image/zxy_euler_angles.hpp"
#include "train_with_cg/collect_image/quaternion.hpp"

using namespace train;

HandPose::HandPose(bool use_quaternion,
                   const std::vector<double>& euler_max, const std::vector<double>& euler_min, double euler_step,
                   const std::vector<double>& finger_max, const std::vector<double>& finger_min, double finger_step)
{
  if(use_quaternion)
  {
    orientation_ = QuaternionPtr(new Quaternion(euler_max, euler_min, euler_step));
  }
  else
  {
    orientation_ = ZXYEulerAnglesPtr(new ZXYEulerAngles(euler_max, euler_min, euler_step));
  }

  fingers_ = FingersPtr(new Fingers(finger_max, finger_min, finger_step));
}

bool HandPose::update()
{
  if(fingers_->update())
  {
    return true;
  }
  else
  {
    if(orientation_->update())
    {
      fingers_->reset();
    }
    else
    {
      return false;
    }
  }

  return true;
}

