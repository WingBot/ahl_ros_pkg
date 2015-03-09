#include <sensor_msgs/image_encodings.h>
#include <tf2_ros/transform_broadcaster.h>
#include "manipit/manipit.hpp"
#include "manipit/exception.hpp"
#include "manipit/hand_detector/rgb_hand_detector.hpp"
#include "manipit/hand_recognizer/deep_learning_depth.hpp"

using namespace manipit;

Manipit::Manipit()
  : received_rgb_(false), received_depth_(false)
{
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  image_transport::ImageTransport it(nh);

  std::string transport;
  local_nh.param<std::string>("image/transport", transport, "raw");
  local_nh.param<bool>("publish_transform", publish_transform_, true);
  local_nh.param<std::string>("frame_id/parent", parent_frame_id_, "world");
  local_nh.param<std::string>("frame_id/child", pose_.header.frame_id, "hand");

  sub_rgb_.subscribe(it, "/camera/rgb/image_rect_color", 1, transport);
  sub_depth_.subscribe(it, "/camera/depth/image_rect", 1, transport);

  sub_rgb_.registerCallback(
    boost::bind(&Manipit::received, this, received_rgb_));
  sub_depth_.registerCallback(
    boost::bind(&Manipit::received, this, received_depth_));

  const int queue_size = 3;

  sync_.reset(
    new Synchronizer(ApproximateTime(queue_size), sub_rgb_, sub_depth_));

  sync_->registerCallback(boost::bind(&Manipit::receivedImages, this, _1, _2));

  rgb_.create(320, 240, CV_8UC3);
  depth_.create(rgb_.rows, rgb_.cols, CV_32FC1);

  detector_ = HandDetectorPtr(new RGBHandDetector());
  recognizer_ = HandRecognizerPtr(new DeepLearningDepth());

  pub_pose_ = nh.advertise<geometry_msgs::PoseStamped>("manipit/hand/pose", 10);
}

void Manipit::receivedImages(const sensor_msgs::ImageConstPtr& msg_rgb,
                             const sensor_msgs::ImageConstPtr& msg_depth)
{
  rgb_ptr_   = cv_bridge::toCvCopy(msg_rgb, sensor_msgs::image_encodings::BGR8);
  depth_ptr_ = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1);

  cv::resize(rgb_ptr_->image, rgb_, rgb_.size());
  cv::resize(depth_ptr_->image, depth_, depth_.size());

  cv::Rect hand_roi;

  if(!detector_->detect(rgb_, depth_, hand_roi))
  {
    ROS_INFO_STREAM("Hand was not detected.");
    return;
  }

  cv::Mat rgb_roi   = rgb_(hand_roi);
  cv::Mat depth_roi = depth_(hand_roi);

  recognizer_->recognizePose(rgb_roi, depth_roi, pose_);

  pub_pose_.publish(pose_);

  if(publish_transform_)
  {
    publishTransform();
  }
}

void Manipit::received(bool& received_image)
{
  received_image = true;
}

void Manipit::publishTransform()
{
  static tf2_ros::TransformBroadcaster br;

  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header = pose_.header;
  transform_stamped.child_frame_id = parent_frame_id_;
  transform_stamped.transform.translation.x = pose_.pose.position.x;
  transform_stamped.transform.translation.y = pose_.pose.position.y;
  transform_stamped.transform.translation.z = pose_.pose.position.z;

  transform_stamped.transform.rotation.x = pose_.pose.orientation.x;
  transform_stamped.transform.rotation.y = pose_.pose.orientation.y;
  transform_stamped.transform.rotation.z = pose_.pose.orientation.z;
  transform_stamped.transform.rotation.w = pose_.pose.orientation.w;

  br.sendTransform(transform_stamped);
}
