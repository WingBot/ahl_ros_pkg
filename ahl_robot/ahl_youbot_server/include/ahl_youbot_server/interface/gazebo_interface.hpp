#ifndef __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP
#define __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <gazebo_msgs/JointStates.h>
#include <gazebo_msgs/ApplyJointEfforts.h>
#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  class GazeboInterface : public Interface
  {
  public:
    GazeboInterface(const std::vector<std::string>& joint_list, const ros::Duration& ctrl_period);

    void getJointStates(Eigen::VectorXd& q);
    void getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq);
    void applyJointEfforts(const Eigen::VectorXd& tau);
  private:
    void jointStatesCB(const gazebo_msgs::JointStates::ConstPtr& msg);

    ros::Subscriber sub_joint_states_;
    ros::Publisher pub_apply_joint_efforts_;

    bool subscribed_joint_states_;

    unsigned int dof_;
    std::vector<std::string> joint_list_;
    std::map<std::string, int> name_to_idx_;
    std::map<std::string, double> q_;
    std::map<std::string, double> dq_;
    gazebo_msgs::ApplyJointEfforts effort_;
    ros::Time start_time_;

  };

}

#endif /* __AHL_YOUBOT_SERVER_GAZEBO_INTERFACE_HPP */
