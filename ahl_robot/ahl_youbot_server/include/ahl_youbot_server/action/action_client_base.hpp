#ifndef __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP
#define __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
#include "ahl_youbot_server/action/action.hpp"

namespace ahl_youbot
{

  class ActionClientBase
  {
  public:
    virtual ~ActionClientBase() {}
    virtual void cancelAllGoals() = 0;
    virtual void cancelGoal() = 0;
    virtual void cancelGoalsAtAndBeforeTime(const ros::Time& time) = 0;
    virtual actionlib::SimpleClientGoalState getState() = 0;
    virtual bool isServerConnected() = 0;
    //template<class GoalType> void sendGoal(const GoalType& goal) { ROS_INFO_STREAM("ActionClientBase::sendGoal"); }
    virtual void sendGoal(void* goal) = 0;
    virtual void stopTrackingGoal() = 0;
    virtual bool waitForResult(const ros::Duration& timeout = ros::Duration(0, 0)) = 0;
    virtual bool waitForServer(const ros::Duration& timeout = ros::Duration(0, 0)) = 0;
  };

  typedef boost::shared_ptr<ActionClientBase> ActionClientBasePtr;
  typedef std::map<Action::Type, ActionClientBasePtr> ActionClientBasePtrMap;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP */
