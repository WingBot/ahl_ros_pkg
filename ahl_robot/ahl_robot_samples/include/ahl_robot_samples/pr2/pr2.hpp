#ifndef __AHL_ROBOT_SAMPLES_PR2_HPP
#define __AHL_ROBOT_SAMPLES_PR2_HPP

#include "ahl_robot_samples/robot_demo.hpp"

namespace ahl_sample
{

  class PR2 : public RobotDemo
  {
  public:
    PR2();

    virtual void init();
    virtual void run();
  private:
    virtual void updateModel(const ros::TimerEvent&);
    virtual void control(const ros::TimerEvent&);
  };

  typedef boost::shared_ptr<PR2> PR2Ptr;
}

#endif /* __AHL_ROBOT_SAMPLES_PR2_HPP */
