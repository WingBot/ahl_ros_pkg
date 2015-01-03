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

#ifndef __CV_WRAPPER_IMAGE_SUBSCRIBER_H
#define __CV_WRAPPER_IMAGE_SUBSCRIBER_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace cv_wrapper
{

  class ImageSubscriber
  {
  public:
    // encoding is defined in sensor_msgs/image_encodings.h
    ImageSubscriber(const std::string& topic, const std::string& encoding);

    bool updated();
    void copyTo(cv::Mat& img);
    void copyHeaderTo(std_msgs::Header& header);

  private:
    void imageCB(const sensor_msgs::ImageConstPtr& msg);

    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;
    std::string encoding_;

    boost::mutex mutex_;
    bool updated_;

    cv_bridge::CvImagePtr cv_ptr_;
    std_msgs::Header header_;
  };

  typedef boost::shared_ptr<ImageSubscriber> ImageSubscriberPtr;
}

#endif /* __CV_WRAPPER_IMAGE_SUBSCRIBER_H */
