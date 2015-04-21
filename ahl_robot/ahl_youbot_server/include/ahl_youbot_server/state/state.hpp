#ifndef __AHL_YOUBOT_SERVER_STATE_HPP
#define __AHL_YOUBOT_SERVER_STATE_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/ahl_robot_actions.hpp"

namespace ahl_youbot
{

  class State
  {
  public:
    enum Type
    {
      DISABLED,
      READY,
      FLOAT,
      MOVE,
      LOCK,
      ALARM,
      STATE_NUM,
    };

    State();
    virtual ~State() {}

    virtual std::string getState()
    {
      return std::string("N/A");
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

  protected:


  };

  typedef boost::shared_ptr<State> StatePtr;
}

#endif /* __AHL_YOUBOT_SERVER_STATE_HPP */
