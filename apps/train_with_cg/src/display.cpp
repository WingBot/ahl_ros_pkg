#include <stdexcept>
#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include <gl_wrapper/exception/exceptions.hpp>
#include "train_with_cg/display.hpp"
#include "train_with_cg/hand_image_collector.hpp"

using namespace train;

void train::display()
{
  try
  {
    gl_wrapper::Render::start();

    static HandImageCollectorPtr hand_image_collector = HandImageCollectorPtr(new HandImageCollector());

    if(hand_image_collector->collect() == false)
    {
      ROS_INFO_STREAM("All hand images were collected.\nShutting down the process ...");
      ros::shutdown();
      exit(0);
    }

    gl_wrapper::Render::end();
  }
  catch(gl_wrapper::FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(gl_wrapper::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
    exit(1);
  }
}
