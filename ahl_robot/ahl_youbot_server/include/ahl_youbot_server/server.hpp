#ifndef __AHL_YOUBOT_SERVER_SERVER_HPP
#define __AHL_YOUBOT_SERVER_SERVER_HPP

#include <map>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "ahl_youbot_server/ahl_robot_srvs.hpp"
#include "ahl_youbot_server/state/state.hpp"
#include "ahl_youbot_server/action/action_server.hpp"
#include "ahl_youbot_server/action/action_client.hpp"

namespace ahl_youbot
{

  class Server
  {
  public:
    Server();

  private:
    bool cancelCB(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    bool floatCB(
      ahl_robot_srvs::Float::Request& req,
      ahl_robot_srvs::Float::Response& res);
    bool setJointCB(
      ahl_robot_srvs::SetJoint::Request& req,
      ahl_robot_srvs::SetJoint::Response& res);
    bool jointSpaceControlCB(
      ahl_robot_srvs::JointSpaceControl::Request& req,
      ahl_robot_srvs::JointSpaceControl::Response& res);
    bool taskSpaceControlCB(
      ahl_robot_srvs::TaskSpaceControl::Request& req,
      ahl_robot_srvs::TaskSpaceControl::Response& res);
    bool taskSpaceHybridControlCB(
      ahl_robot_srvs::TaskSpaceHybridControl::Request& req,
      ahl_robot_srvs::TaskSpaceHybridControl::Response& res);

    std::map<State::Type, StatePtr> state_;
    State::Type state_type_;

    ros::ServiceServer ros_server_cancel_;
    ros::ServiceServer ros_server_float_;
    ros::ServiceServer ros_server_set_joint_;
    ros::ServiceServer ros_server_joint_space_control_;
    ros::ServiceServer ros_server_task_space_control_;
    ros::ServiceServer ros_server_task_space_hybrid_control_;

    ActionServerPtr action_server_;
    ActionClientBasePtrMap action_client_;
  };

  typedef boost::shared_ptr<Server> ServerPtr;
};

#endif /* __AHL_YOUBOT_SERVER_SERVER_HPP */
