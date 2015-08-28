#include <Eigen/Dense>
#include <ros/ros.h>
#include "train_with_cg/auto_encoder.hpp"

using namespace train;

AutoEncoder::AutoEncoder()
{
  ros::NodeHandle local_nh("train");

  bool data_in_image  = false;
  bool data_out_image = false;

  // TODO

  std::string config_path;
  std::string data_in_path;
  std::string data_out_path;
  std::string result_path;

  // TODO

  config_ = nn::ConfigPtr(new nn::Config());
  data_   = nn::TrainingDataPtr(new nn::TrainingData());
  nn_     = nn::NeuralNetworkPtr(new nn::NeuralNetwork());

  config_->init(config_path);
  data_->init(data_in_path, data_out_path);
}

