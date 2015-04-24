#ifndef __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP
#define __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP

#include <actionlib/client/terminal_state.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace ahl_youbot
{

  class ActionClientBase
  {
  public:
    virtual ~ActionClientBase() {}
    virtual void cancelAllGoals() = 0;
    virtual void cancelGoal() = 0;
    virtual void cancelGoalsAtAndBeforeTime(const ros::Time& time) = 0;
    template<class GoalType> GoalType getResult()
    {
      GoalType empty;
      return empty;
    }
    virtual actionlib::SimpleClientGoalState getState() = 0;
    virtual bool isServerConnected() = 0;
    template<class GoalType> void sendGoal(const GoalType& goal) {}
    virtual void stopTrackingGoal() = 0;
    virtual bool waitForResult(const ros::Duration& timeout = ros::Duration(0, 0)) = 0;
    virtual bool waitForServer(const ros::Duration& timeout = ros::Duration(0, 0)) = 0;
  };

  typedef boost::shared_ptr<ActionClientBase> ActionClientBasePtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_CLIENT_BASE_HPP */
