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

#ifndef __MANIPIT_MANIPIT_HPP
#define __MANIPIT_MANIPIT_HPP

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include "manipit/hand_detector/hand_detector.hpp"
#include "manipit/hand_recognizer/hand_recognizer.hpp"

namespace manipit
{

  class Manipit
  {
  public:
    Manipit();

  private:
    void receivedImages(const sensor_msgs::ImageConstPtr& msg_rgb,
                        const sensor_msgs::ImageConstPtr& msg_depth);
    void received(bool& received_image);
    void publishTransform();

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproximateTime;
    typedef message_filters::Synchronizer<ApproximateTime> Synchronizer;
    typedef boost::shared_ptr<Synchronizer> SynchronizerPtr;

    image_transport::SubscriberFilter sub_rgb_;
    image_transport::SubscriberFilter sub_depth_;
    SynchronizerPtr sync_;

    cv_bridge::CvImagePtr rgb_ptr_;
    cv_bridge::CvImagePtr depth_ptr_;

    cv::Mat rgb_;
    cv::Mat depth_;

    bool received_rgb_;
    bool received_depth_;

    HandDetectorPtr detector_;
    HandRecognizerPtr recognizer_;

    geometry_msgs::PoseStamped pose_;

    ros::Publisher pub_pose_;

    bool publish_transform_;
    std::string parent_frame_id_;
  };

  typedef boost::shared_ptr<Manipit> ManipitPtr;
}

#endif /* __MANIPIT_MANIPIT_HPP */
