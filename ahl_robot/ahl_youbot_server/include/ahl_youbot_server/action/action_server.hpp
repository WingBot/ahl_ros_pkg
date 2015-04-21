#ifndef __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP
#define __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP

#include <boost/shared_ptr.hpp>

#include "ahl_youbot_server/ahl_robot_actions.hpp"

#include <map>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class ActionServer
  {
  public:
    ActionServer();

    void start(Action::Type type);
    void cancel(Action::Type type);
    void shutdown(Action::Type type);
    bool isActive(Action::Type type);

  private:
    std::map<Action::Type, ActionPtr> action_;

  };

  typedef boost::shared_ptr<ActionServer> ActionServerPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_SERVER_HPP */
