#ifndef __AHL_YOUBOT_SERVER_MOVE_HPP
#define __AHL_YOUBOT_SERVER_MOVE_HPP

#include "ahl_youbot_server/state/state.hpp"

namespace ahl_youbot
{

  class Move : public State
  {
  public:
    Move(const ActionServerPtr& action_server)
      : State(action_server) {}

    virtual std::string getState()
    {
      return std::string("Move");
    }

  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_MOVE_HPP */
