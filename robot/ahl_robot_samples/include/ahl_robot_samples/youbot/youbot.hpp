#ifndef __AHL_ROBOT_SAMPLES_YOUBOT_HPP
#define __AHL_ROBOT_SAMPLES_YOUBOT_HPP

#include "ahl_robot_samples/robot_demo.hpp"
#include "ahl_robot_samples/youbot/youbot_param.hpp"

namespace ahl_sample
{

  class YouBot : public RobotDemo
  {
  public:
    YouBot();

    virtual void init();
    virtual void run();
  private:
    virtual void updateModel(const ros::TimerEvent&);
    virtual void control(const ros::TimerEvent&);
    void updateWheels(const ros::TimerEvent&);

    YouBotParamPtr param_;

    TaskPtr gravity_compensation_;
    TaskPtr joint_control_;
    TaskPtr arm_position_control_;
    TaskPtr arm_orientation_control_;
    TaskPtr base_position_control_;
    TaskPtr base_orientation_control_;
    GazeboInterfacePtr gazebo_interface_wheel_;

    ros::Timer timer_update_wheels_;
    Eigen::VectorXd q_base_;
    Eigen::VectorXd tau_base_;
  };

  typedef boost::shared_ptr<YouBot> YouBotPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_YOUBOT_HPP */
