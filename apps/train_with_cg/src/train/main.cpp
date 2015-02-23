#include <ros/ros.h>
#include "train_with_cg/train/train.hpp"
#include "train_with_cg/exceptions.hpp"

int main(int argc, char** argv)
{
  try
  {
    ros::init(argc, argv, "train_with_cg");
    ros::NodeHandle nh;

    train::TrainPtr train = train::TrainPtr(new train::Train());
    train->start();
    train->save();
  }
  catch(train::Exception& e)
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
}
