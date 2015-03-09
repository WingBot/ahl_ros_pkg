#include <ros/ros.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "train_with_cg/auto_encoder/auto_encoder.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

AutoEncoder::AutoEncoder()
{
  ros::NodeHandle local_nh("~");

  local_nh.param<std::string>("path/config", config_full_path_,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/yaml/auto_encoder0.yaml");
  local_nh.param<std::string>("path/data_in", data_in_path_,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/data/depth");
  local_nh.param<std::string>("data_name", data_name_, "image");
  local_nh.param<std::string>("extension", extension_, "pgm");
  local_nh.param<std::string>("path/result", result_full_path_,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/results/auto_encoder/auto_encoder0_result.yaml");

  local_nh.param<bool>("use_image_data", use_image_data_, true);
}

void AutoEncoder::init()
{
  config_ = nn::ConfigPtr(new nn::Config());
  data_ = nn::TrainingDataPtr(new nn::TrainingData());
  nn_ = nn::NeuralNetworkPtr(new nn::NeuralNetwork());

  if(use_image_data_)
  {
    unsigned long idx = 0;

    cv::Mat image;
    while(1)
    {
      std::stringstream name;
      name << data_in_path_ << "/" << data_name_ << idx << "." << extension_;

      image = cv::imread(name.str(), CV_8UC1);

      if(!image.data)
      {
        break;
      }

      Eigen::MatrixXd input(image.rows * image.cols, 1);
      Eigen::MatrixXd output(image.rows * image.cols, 1);

      for(unsigned int y = 0; y < image.rows; ++y)
      {
        uchar* ptr = image.ptr<uchar>(y);

        for(unsigned int x = 0; x < image.cols; ++x)
        {
          input.coeffRef(x + y * image.cols, 0)  = static_cast<double>(ptr[x] / 255.0);
          output.coeffRef(x + y * image.cols, 0) = input.coeff(x + y * image.cols, 0);
        }
      }

      data_->add(input, output);

      cv::Mat resized_image(image.rows * 10, image.cols * 10, CV_8UC1);
      cv::resize(image, resized_image, resized_image.size());

      cv::imshow("image", resized_image);

      if(cv::waitKey(2) == 27)
        exit(0);

      ++idx;
    }
  }
  else
  {

  }

  config_->init(config_full_path_);
  nn_->init(config_);
  config_->print();
}

void AutoEncoder::train()
{
  if(config_->enableBackPropagation())
  {
    ROS_INFO_STREAM("Resulting file could be saved in " + result_full_path_ + ".");
    nn_->train(data_, result_full_path_);
  }
  else
  {
    throw train::Exception("AutoEncoder", "config_->enableBackPropagation() is false.");
  }
}
