#ifndef __AHL_YOUBOT_SERVER_LOCK_HPP
#define __AHL_YOUBOT_SERVER_LOCK_HPP

#include "ahl_youbot_server/state/state.hpp"

namespace ahl_youbot
{

  class Lock : public State
  {
  public:
    Lock(State::Type& state_type, const ActionServerPtr& server)
      : State(state_type, server) {}

    virtual std::string getState()
    {
      return std::string("Lock");
    }
  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_LOCK_HPP */
