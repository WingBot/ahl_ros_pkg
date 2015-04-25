#ifndef __AHL_YOUBOT_SERVER_READY_HPP
#define __AHL_YOUBOT_SERVER_READY_HPP

#include "ahl_youbot_server/state/state.hpp"

namespace ahl_youbot
{

  class Ready : public State
  {
  public:
    Ready(State::Type& state_type, const ActionServerPtr& server, const ActionClientBasePtrMap& client)
      : State(state_type, server, client) {}

    virtual std::string getState()
    {
      return std::string("Ready");
    }

    virtual bool callFloat(
      ahl_robot_srvs::Float::Request& req,
      ahl_robot_srvs::Float::Response& res);
    virtual bool callSetJoint(
      ahl_robot_srvs::SetJoint::Request& req,
      ahl_robot_srvs::SetJoint::Response& res);
    virtual bool callJointSpaceControl(
      ahl_robot_srvs::JointSpaceControl::Request& req,
      ahl_robot_srvs::JointSpaceControl::Response& res);
    virtual bool callTaskSpaceControl(
      ahl_robot_srvs::TaskSpaceControl::Request& req,
      ahl_robot_srvs::TaskSpaceControl::Response& res);
    virtual bool callTaskSpaceHybridControl(
      ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
      ahl_robot_srvs::TaskSpaceHybridControl::Response& res);

  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_READY_HPP */
