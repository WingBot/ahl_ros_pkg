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

#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include "train_with_cg/collect_image/display.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "train_with_cg");
  ros::NodeHandle nh;

  try
  {
    gl_wrapper::RenderPtr render = gl_wrapper::RenderPtr(new gl_wrapper::Render(argc, argv));
    render->start(train::display);
  }
  catch(gl_wrapper::FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(gl_wrapper::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
    exit(1);
  }

  return 0;
}
