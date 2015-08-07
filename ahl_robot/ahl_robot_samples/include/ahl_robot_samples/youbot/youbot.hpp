#ifndef __AHL_ROBOT_SAMPLES_YOUBOT_HPP
#define __AHL_ROBOT_SAMPLES_YOUBOT_HPP

#include "ahl_robot_samples/robot_demo.hpp"

namespace ahl_sample
{

  class YouBot : public RobotDemo
  {
  public:
    YouBot();

    virtual void init() {}
    virtual void run() {}
  private:
    virtual void updateModel(const ros::TimerEvent&) {}
    virtual void control(const ros::TimerEvent&) {}



  };

  typedef boost::shared_ptr<YouBot> YouBotPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_YOUBOT_HPP */
