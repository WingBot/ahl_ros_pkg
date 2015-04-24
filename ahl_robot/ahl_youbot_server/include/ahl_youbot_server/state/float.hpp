#ifndef __AHL_YOUBOT_SERVER_FLOAT_HPP
#define __AHL_YOUBOT_SERVER_FLOAT_HPP

#include "ahl_youbot_server/state/state.hpp"

namespace ahl_youbot
{

  class Float : public State
  {
  public:
    Float(const ActionServerPtr& action_server)
      : State(action_server) {}

    virtual std::string getState()
    {
      return std::string("Float");
    }
  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_FLOAT_HPP */
