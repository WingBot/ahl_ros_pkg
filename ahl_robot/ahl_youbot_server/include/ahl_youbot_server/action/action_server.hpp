#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>

#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer(
      const std::string& robot_name, const std::string& mnp_name, const std::string& yaml,
      double period, double servo_period, bool use_real_robot);

    void start();
    void stop();
    void changeActionTo(Action::Type action_type)
    {
      action_type_ = action_type;
    }

    const std::string& getActionName(Action::Type type)
    {
      return action_[type]->getActionName();
    }

  private:
    void timerCB(const ros::TimerEvent&);
    void servoTimerCB(const ros::TimerEvent&);

    bool updated_model_;

    std::map<Action::Type, ActionPtr> action_;
    Action::Type action_type_;

    InterfacePtr interface_;
    ahl_robot::RobotPtr robot_;
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;

    boost::mutex mutex_;
    ros::Timer timer_;
    ros::Timer servo_timer_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
