#ifndef __AHL_YOUBOT_SERVER_ACTION_IMPL_HPP
#define __AHL_YOUBOT_SERVER_ACTION_IMPL_HPP

#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  template<class GoalType>
  class ActionImpl : public Action
  {
  public:
    ActionImpl(const std::string& action_name)
      : Action(action_name) {}
    virtual ~ActionImpl() {}

  private:
  };

}

#endif /* __AHL_YOUBOT_SERVER_ACTION_IMPL_HPP */
