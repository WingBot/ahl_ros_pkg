#ifndef __AHL_YOUBOT_SAMPLE_VIEWER_HPP
#define __AHL_YOUBOT_SAMPLE_VIEWER_HPP

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

namespace ahl_youbot
{

  class Viewer
  {
  public:
    Viewer();

  private:
    void timerCB(const ros::TimerEvent&);

    typedef boost::shared_ptr<youbot::YouBotBase> YouBotBasePtr;
    typedef boost::shared_ptr<youbot::YouBotManipulator> YouBotManipulatorPtr;

    YouBotBasePtr base_;
    YouBotManipulatorPtr manipulator_;

    ros::Timer timer_;
  };

}

#endif /* __AHL_YOUBOT_SAMPLE_VIEWER_HPP */
