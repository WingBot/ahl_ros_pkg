#ifndef __AHL_YOUBOT_SERVER_ALARM_HPP
#define __AHL_YOUBOT_SERVER_ALARM_HPP

#include "ahl_youbot_server/state/state.hpp"

namespace ahl_youbot
{

  class Alarm : public State
  {
  public:
    Alarm(const ActionServerPtr& server, const ActionClientBasePtrMap& client)
      : State(server, client) {}

    virtual std::string getState()
    {
      return std::string("Alarm");
    }

  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_ALARM_HPP */
