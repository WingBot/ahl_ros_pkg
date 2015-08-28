#ifndef __AHL_ROBOT_SAMPLES_PR2_HPP
#define __AHL_ROBOT_SAMPLES_PR2_HPP

#include "ahl_robot_samples/robot_demo.hpp"
#include "ahl_robot_samples/pr2/pr2_param.hpp"

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

    Pr2ParamPtr param_;

    TaskPtr gravity_compensation_;
    TaskPtr joint_limit_l_;
    TaskPtr joint_limit_r_;
    TaskPtr joint_control_l_;
    TaskPtr joint_control_r_;
    TaskPtr position_control_l_;
    TaskPtr position_control_r_;
    TaskPtr orientation_control_l_;
    TaskPtr orientation_control_r_;
  };

  typedef boost::shared_ptr<PR2> PR2Ptr;
}

#endif /* __AHL_ROBOT_SAMPLES_PR2_HPP */
