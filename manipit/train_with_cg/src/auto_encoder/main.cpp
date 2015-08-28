#include <stdexcept>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include "train_with_cg/auto_encoder/auto_encoder.hpp"
#include "train_with_cg/exceptions.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "auto_encoder");
    ros::NodeHandle nh;

    train::AutoEncoderPtr auto_encoder = train::AutoEncoderPtr(new train::AutoEncoder());
    auto_encoder->init();
    auto_encoder->train();
    ros::spin();
  }
  catch(cv::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(train::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(nn::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
  }

  return 0;
}


