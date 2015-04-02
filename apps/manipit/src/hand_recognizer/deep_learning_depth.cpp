#include <ros/ros.h>
#include "manipit/hand_recognizer/deep_learning_depth.hpp"

using namespace manipit;

DeepLearningDepth::DeepLearningDepth()
{
  ros::NodeHandle local_nh("deep_learning_depth");

  config_ = nn::ConfigPtr(new nn::Config());
  nn_ = nn::NeuralNetworkPtr(new nn::NeuralNetwork());

  std::string config_path;
  local_nh.param<std::string>("deep_learning_depth/config_path", config_path, "");

  //config_->init(config_path);
  //nn_->init(config_);
}

bool DeepLearningDepth::recognizePose(const cv::Mat& rgb, const cv::Mat& depth, geometry_msgs::PoseStamped& pose)
{
  return true;
}
