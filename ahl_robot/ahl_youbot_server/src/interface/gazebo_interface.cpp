#include "ahl_youbot_server/exceptions.hpp"
#include "ahl_youbot_server/interface/gazebo_interface.hpp"

using namespace ahl_youbot;

GazeboInterface::GazeboInterface(const std::vector<std::string>& joint_list, const ros::Duration& ctrl_period)
  : subscribed_joint_states_(false)
{
  dof_ = joint_list.size();
  for(unsigned int i = 0; i < dof_; ++i)
  {
    name_to_idx_[joint_list[i]] = i;
    joint_list_.push_back(joint_list[i]);
    q_[joint_list[i]]   = 0.0;
    dq_[joint_list[i]]  = 0.0;
    effort_.name.push_back(joint_list[i]);
  }
  effort_.effort.resize(effort_.name.size());
  effort_.start_time = ros::Time(0);
  effort_.duration = ctrl_period;

  ros::NodeHandle nh;
  sub_joint_states_ = nh.subscribe("/gazebo/joint_states", 10, &GazeboInterface::jointStatesCB, this);
  pub_apply_joint_efforts_ = nh.advertise<gazebo_msgs::ApplyJointEfforts>("/gazebo/apply_joint_efforts", 10);
}

void GazeboInterface::getJointStates(Eigen::VectorXd& q)
{
  if(!subscribed_joint_states_)
  {
    return;
  }

  if(q.rows() != joint_list_.size())
  {
    std::stringstream msg;
    msg << "q.rows() != joint_list.size()" << std::endl
        << "  q.rows          : " << q.rows() << std::endl
        << "  joint_list.size : " << joint_list_.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::getJointStates", msg.str());
  }

  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(q_.find(joint_list_[i]) != q_.end())
    {
      q.coeffRef(i) = q_[joint_list_[i]];
    }
  }
}

void GazeboInterface::getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq)
{
  if(!subscribed_joint_states_)
  {
    return;
  }

  if(q.rows() != dq.rows())
  {
    std::stringstream msg;
    msg << "q.rows() != dq.rows()" << std::endl
        << "  q.rows  : " << q.rows() << std::endl
        << "  dq.rows : " << dq.rows();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::getJointStates", msg.str());
  }

  if(q.rows() != joint_list_.size())
  {
    std::stringstream msg;
    msg << "q.rows() != joint_list.size()" << std::endl
        << "  q.rows          : " << q.rows() << std::endl
        << "  joint_list.size : " << joint_list_.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::getJointStates", msg.str());
  }

  for(unsigned int i = 0; i < joint_list_.size(); ++i)
  {
    if(q_.find(joint_list_[i]) != q_.end())
    {
      q.coeffRef(i) = q_[joint_list_[i]];
    }

    if(dq_.find(joint_list_[i]) != dq_.end())
    {
      dq.coeffRef(i) = dq_[joint_list_[i]];
    }
  }
}

void GazeboInterface::applyJointEfforts(const Eigen::VectorXd& tau)
{
  if(tau.rows() != effort_.name.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != effort_.name.size()" << std::endl
        << "  tau.rows          : " << tau.rows() << std::endl
        << "  effort_.name.size : " << effort_.name.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }
  else if(tau.rows() != effort_.effort.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != effort_.effort.size()" << std::endl
        << "  tau.rows            : " << tau.rows() << std::endl
        << "  effort_.effort.size : " << effort_.effort.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }
  else if(tau.rows() != joint_list_.size())
  {
    std::stringstream msg;
    msg << "tau.rows() != joint_list_.size()" << std::endl
        << "  tau.rows            : " << tau.rows() << std::endl
        << "  joint_list_.size : " << joint_list_.size();
    throw ahl_youbot::Exception("ahl_youbot::GazeboInterface::applyJointEfforts", msg.str());
  }

  for(unsigned int i = 0; i < tau.rows(); ++i)
  {
    if(name_to_idx_.find(joint_list_[i]) == name_to_idx_.end())
      continue;

    int idx = name_to_idx_[joint_list_[i]];

    if(idx > effort_.effort.size() - 1)
      continue;

    effort_.effort[idx] = tau.coeff(i);
  }
}

void GazeboInterface::jointStatesCB(const gazebo_msgs::JointStates::ConstPtr& msg)
{
  subscribed_joint_states_ = true;

  for(unsigned int i = 0; i < msg->name.size(); ++i)
  {
    q_[msg->name[i]]  = msg->q[i];
    dq_[msg->name[i]] = msg->dq[i];
  }
}
