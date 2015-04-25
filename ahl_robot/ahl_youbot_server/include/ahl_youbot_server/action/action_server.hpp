#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <youbot_driver/youbot/YouBotBase.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>

#include "ahl_youbot_server/ahl_robot_actions.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/youbot/youbot.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer(bool use_real_robot);

    void start(Action::Type type);
    void preempt(Action::Type type);
    void cancel(Action::Type type);
    void cancel();
    void shutdown(Action::Type type);
    bool isActive(Action::Type type);

    const std::string& getActionName(Action::Type type)
    {
      return action_[type]->getActionName();
    }

  private:
    std::map<Action::Type, ActionPtr> action_;
    std::map<Action::Type, ros::Publisher> canceller_;
    YouBotPtr youbot_;
  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
