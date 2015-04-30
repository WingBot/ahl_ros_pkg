#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/youbot/youbot.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer(bool use_real_robot);

    const std::string& getActionName(Action::Type type)
    {
      return action_[type]->getActionName();
    }

  private:
    void timerCB(const ros::TimerEvent&);

    std::map<Action::Type, ActionPtr> action_;
    std::map<Action::Type, ros::ServiceServer> ros_server_;
    Action::Type action_type_;

    YouBotPtr youbot_;

    ros::Timer timer_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
