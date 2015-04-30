#ifndef __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP
#define __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP

#include <map>

#include "ahl_youbot_server/ahl_robot_actions.hpp"
#include "ahl_youbot_server/action/action.hpp"
#include "ahl_youbot_server/action/action_client_base.hpp"

namespace ahl_youbot
{

  template<typename ActionType, typename GoalType>
  class ActionClient : public ActionClientBase
  {
  public:
    ActionClient(const std::string& action_name)
    {
      client_ = boost::shared_ptr< actionlib::SimpleActionClient<ActionType> >(
        new actionlib::SimpleActionClient<ActionType>(action_name, true));
    }

    void cancelAllGoals()
    {
      client_->cancelAllGoals();
    }

    void cancelGoal()
    {
      client_->cancelGoal();
    }

    void cancelGoalsAtAndBeforeTime(const ros::Time& time)
    {
      client_->cancelGoalsAtAndBeforeTime(time);
    }

    actionlib::SimpleClientGoalState getState()
    {
      return client_->getState();
    }

    bool isServerConnected()
    {
      return client_->isServerConnected();
    }

    void sendGoal(void* goal)
    {
      client_->sendGoal(*(static_cast<GoalType*>(goal)));
    }

    void stopTrackingGoal()
    {

    }

    bool waitForResult(const ros::Duration& timeout = ros::Duration(0, 0))
    {
      return client_->waitForResult(timeout);
    }

    bool waitForServer(const ros::Duration& timeout = ros::Duration(0, 0))
    {
      return client_->waitForServer(timeout);
    }

  private:
    boost::shared_ptr< actionlib::SimpleActionClient<ActionType> > client_;
  };

  template<class ActionType,class GoalType>
  struct ActionClientPtr
  {
    typedef boost::shared_ptr< ActionClientPtr<ActionType, GoalType> > client;
  };
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_CLIENT_HPP */
