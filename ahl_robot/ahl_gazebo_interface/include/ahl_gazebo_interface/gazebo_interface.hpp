#ifndef __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP
#define __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP

#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <gazebo_msgs/JointStates.h>
#include <gazebo_msgs/ApplyJointEfforts.h>

namespace ahl_gazebo_if
{

  static const std::string TOPIC_PUB = "/gazebo/apply_joint_efforts";
  static const std::string TOPIC_SUB = "/gazebo/joint_states";

  class GazeboInterface
  {
  public:
    GazeboInterface();

    void addJoint(const std::string& name);
    void setDuration(double duration);
    void connect();
    bool subscribed()
    {
      return subscribed_;
    }

    void applyJointEfforts(const Eigen::VectorXd& tau);

    const Eigen::VectorXd& getJointStates() const
    {
      return q_;
    }

  private:
    void subscribe(const gazebo_msgs::JointStates::ConstPtr& msg);

    std::map<std::string, double> q_map_;
    std::map<std::string, int> name_to_idx_;
    std::vector<std::string> name_list_;
    unsigned int joint_num_;

    Eigen::VectorXd q_;
    gazebo_msgs::ApplyJointEfforts effort_;
    ros::Duration duration_;
    ros::Publisher pub_effort_;
    ros::Subscriber sub_joint_states_;
    bool subscribed_;
  };

  typedef boost::shared_ptr<GazeboInterface> GazeboInterfacePtr;
}

#endif /* __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP */
