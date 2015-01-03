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

#include <sensor_msgs/image_encodings.h>
#include "cv_wrapper/image_subscriber.hpp"
#include "cv_wrapper/exceptions.hpp"

using namespace cv_wrapper;

ImageSubscriber::ImageSubscriber(const std::string& topic, const std::string& encoding)
  : updated_(false), encoding_(encoding)
{
  ros::NodeHandle nh;
  it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh));

  sub_ = it_->subscribe(topic, 1, &ImageSubscriber::imageCB, this);
}

bool ImageSubscriber::updated()
{
  boost::mutex::scoped_lock lock(mutex_);
  return updated_;
}

void ImageSubscriber::copyTo(cv::Mat& img)
{
  boost::mutex::scoped_lock lock(mutex_);
  img = cv_ptr_->image;
}

void ImageSubscriber::copyHeaderTo(std_msgs::Header& header)
{
  boost::mutex::scoped_lock lock(mutex_);
  header = header_;
}

void ImageSubscriber::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);
  try
  {
    cv_ptr_ = cv_bridge::toCvCopy(msg, encoding_);
    header_ = msg->header;
    updated_ = true;
  }
  catch(cv_bridge::Exception& e)
  {
    throw Exception("ImageSubscriber::imageCB", e.what());
  }
}
