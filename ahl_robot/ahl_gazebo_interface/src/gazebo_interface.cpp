#include "ahl_gazebo_interface/gazebo_interface.hpp"
#include "ahl_gazebo_interface/exception.hpp"

using namespace ahl_gazebo_if;

GazeboInterface::GazeboInterface()
  : duration_(ros::Duration(0.1)), subscribed_(false)
{
}

void GazeboInterface::addJoint(const std::string& name)
{
  q_map_[name] = 0.0;
  if(name_to_idx_.find(name) == name_to_idx_.end())
  {
    unsigned int size = name_to_idx_.size();
    name_to_idx_[name] = size;
    name_list_.push_back(name);
  }
}

void GazeboInterface::setDuration(double duration)
{
  duration_ = ros::Duration(duration);
}

void GazeboInterface::connect()
{
  joint_num_ = q_map_.size();
  q_ = Eigen::VectorXd::Zero(joint_num_);
  effort_.start_time = ros::Time(0);
  effort_.duration = duration_;
  effort_.effort.resize(joint_num_);
  effort_.name.resize(joint_num_);
  std::map<std::string, int>::iterator it;
  for(it = name_to_idx_.begin(); it != name_to_idx_.end(); ++it)
  {
    effort_.name[it->second] = it->first;
  }

  ros::NodeHandle nh;
  pub_effort_ = nh.advertise<gazebo_msgs::ApplyJointEfforts>(TOPIC_PUB, 10);
  sub_joint_states_ = nh.subscribe(TOPIC_SUB, 10, &GazeboInterface::subscribe, this);
}

void GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  if(tau.rows() != name_list_.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != name_list_.size()" << std::endl
        << "  tau.rows()        : " << tau.rows() << std::endl
        << "  name_list_.size() : " << name_list_.size();

    throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEfforts", msg.str());
  }

  for(unsigned int i = 0; i < tau.rows(); ++i)
  {
    if(name_to_idx_.find(name_list_[i]) == name_to_idx_.end())
    {
      std::stringstream msg;
      msg << name_list_[i] << " was not found in name_to_idx_.";
      throw ahl_gazebo_if::Exception("ahl_gazebo_if::GazeboInterface::applyJointEffots", msg.str());
    }

    effort_.effort[name_to_idx_[name_list_[i]]] = tau.coeff(i);
  }

  pub_effort_.publish(effort_);
}

void GazeboInterface::subscribe(const gazebo_msgs::JointStates::ConstPtr& msg)
{
  for(unsigned int i = 0; i < msg->name.size(); ++i)
  {
    q_map_[msg->name[i]] = msg->q[i];
  }

  for(unsigned int i = 0; i < name_list_.size(); ++i)
  {
    if(q_map_.find(name_list_[i]) != q_map_.end())
    {
      q_.coeffRef(i) = q_map_[name_list_[i]];
    }
  }

  subscribed_ = true;
}
