#ifndef __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP
#define __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>

#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_robot_controller/exception.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/tasks.hpp>

using namespace ahl_gazebo_if;
using namespace ahl_robot;
using namespace ahl_ctrl;

namespace ahl_sample
{

  class RobotDemo
  {
  public:
    RobotDemo() : joint_updated_(false), model_updated_(false) {}
    virtual ~RobotDemo() {}
    virtual void init() = 0;
    virtual void run()  = 0;

  protected:
    virtual void initRobot(const std::string& name, const std::string& yaml)
    {
      robot_ = RobotPtr(new Robot(name));
      ParserPtr parser = ParserPtr(new Parser());
      parser->load(yaml, robot_);
    }

    virtual void updateModel(const ros::TimerEvent&) = 0;
    virtual void control(const ros::TimerEvent&) = 0;

    boost::mutex mutex_;
    RobotPtr robot_;
    RobotControllerPtr controller_;
    bool joint_updated_;
    bool model_updated_;
    ros::Timer timer_update_model_;
    ros::Timer timer_control_;
    TfPublisherPtr tf_pub_;
    ahl_gazebo_if::GazeboInterfacePtr gazebo_interface_;
  };

}

#endif /* __AHL_ROBOT_SAMPLES_ROBOT_DEMO_HPP */
