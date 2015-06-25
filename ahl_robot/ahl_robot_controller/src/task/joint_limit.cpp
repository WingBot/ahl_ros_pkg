#include "ahl_robot_controller/task/joint_limit.hpp"

using namespace ahl_ctrl;

JointLimit::JointLimit(const ahl_robot::ManipulatorPtr& mnp, double threshold)
  : kp_(0.0), kv_(0.0), threshold_(threshold)
{
  mnp_ = mnp;

  q_max_.resize(mnp_->link.size());
  q_min_.resize(mnp_->link.size());

  for(unsigned int i = 0; i < mnp_->link.size(); ++i)
  {
    q_max_.coeffRef(i) = mnp_->link[i]->q_max;
    q_min_.coeffRef(i) = mnp_->link[i]->q_min;
  }

  N_ = Eigen::MatrixXd::Identity(mnp_->dof, mnp_->dof);
}

void JointLimit::computeGeneralizedForce(Eigen::VectorXd& tau)
{
  tau = Eigen::VectorXd::Zero(mnp_->dof);

  kp_ = 10.0;
  kv_ = 1.0;

  for(unsigned int i = 0; i < mnp_->q.rows(); ++i)
  {
    double q_max_diff = q_max_.coeff(i) - mnp_->q.coeff(i);
    double q_min_diff = q_min_.coeff(i) - mnp_->q.coeff(i);

    if(q_max_diff < threshold_)
    {
      tau.coeffRef(i) += -kp_ * q_max_diff - kv_ * mnp_->dq.coeff(i);
      N_.coeffRef(i, i) = 0;
    }
    if(-q_min_diff < threshold_)
    {
      tau.coeffRef(i) += -kp_ * q_min_diff - kv_ * mnp_->dq.coeff(i);
      N_.coeffRef(i, i) = 0;
    }
  }
}
