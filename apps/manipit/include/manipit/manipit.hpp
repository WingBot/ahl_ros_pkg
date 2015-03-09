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
