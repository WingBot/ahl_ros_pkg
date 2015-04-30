#ifndef __AHL_YOUBOT_SERVER_ACTION_HPP
#define __AHL_YOUBOT_SERVER_ACTION_HPP

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

namespace ahl_youbot
{

  class Action
  {
  public:
    enum Type
    {
      FLOAT,
      SET_JOINT,
      JOINT_SPACE_CONTROL,
      TASK_SPACE_CONTROL,
      TASK_SPACE_HYBRID_CONTROL,
      ACTION_NUM,
    };

    Action(const std::string& action_name)
      : action_name_(action_name) {}
    virtual ~Action() {}

    virtual void execute(void* goal) = 0;

    virtual const std::string& getActionName() const
    {
      return action_name_;
    }

  protected:
    const ros::NodeHandle& getNodeHandle() const
    {
      return nh_;
    }

  private:
    ros::NodeHandle nh_;
    std::string action_name_;
  };

  typedef boost::shared_ptr<Action> ActionPtr;
}

#endif /* __AHL_YOUBOT_SERVER_ACTION_HPP */
