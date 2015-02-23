#include <stdexcept>
#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include <gl_wrapper/exception/exceptions.hpp>
#include "train_with_cg/collect_image/display.hpp"
#include "train_with_cg/collect_image/hand_image_collector.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

void train::display()
{
  try
  {
    gl_wrapper::Render::start();
    static HandImageCollectorPtr hand_image_collector = HandImageCollectorPtr(new HandImageCollector());

    hand_image_collector->collect();

    if(hand_image_collector->finished())
    {
      ROS_INFO_STREAM("All hand images were collected.\nShutting down the process ...");
      ros::shutdown();
      exit(0);
    }

    gl_wrapper::Render::end();
  }
  catch(train::FatalException& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(train::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
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
