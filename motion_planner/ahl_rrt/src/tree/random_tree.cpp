#include <ctime>
#include <ros/ros.h>
#include "ahl_rrt/exception.hpp"
#include "ahl_rrt/Edge3D.h"
#include "ahl_rrt/third_party/MT.h"
#include "ahl_rrt/tree/random_tree.hpp"

using namespace ahl_rrt;

void RandomTree::init(const ParamPtr& param, const Eigen::VectorXd& init_x)
{
  root_ = VertexPtr(new Vertex());

  param_ = param;
  root_->setX(init_x);

#ifdef ENABLE_VISUALIZATION
  ros::NodeHandle nh;
  pub_ = nh.advertise<ahl_rrt::Edge3D>("ahl_rrt/edge", 10000);
#endif
}

void RandomTree::build(const Eigen::VectorXd& dst_x)
{
  if(root_->getX().rows() != dst_x.rows())
  {
    std::stringstream msg;

    msg << "Sizes of initial state and goal state are different." << std::endl
        << "  initial state size : " << root_->getX().rows() << std::endl
        << "  goal state size    : " << dst_x.rows();

    throw ahl_rrt::Exception("RandomTree::Build", msg.str());
  }

  unsigned long cnt = 0;

  Eigen::VectorXd diff = param_->max - param_->min;
  Eigen::VectorXd bias = Eigen::VectorXd::Constant(dst_x.rows(), 1.0);

  init_genrand(static_cast<unsigned int>(std::time(NULL)));

  while(cnt < param_->max_iterations && ros::ok())
  {
    std::srand(static_cast<unsigned int>(genrand_int32()));

    Eigen::VectorXd x_rand = 0.5 * (Eigen::VectorXd::Random(dst_x.rows()) + bias);
    x_rand = x_rand.array() * diff.array() + param_->min.array();

    VertexPtr v_nearest = this->getNearestVertex(x_rand, root_);
    VertexPtr child = this->getNewVertex(x_rand, v_nearest);
    v_nearest->addChild(child);

#ifdef ENABLE_VISUALIZATION
    this->publish(v_nearest, child);
    ros::Duration(0.001).sleep();
#endif

    if(this->reachedToGoal(v_nearest->getX(), dst_x))
    {
      break;
    }

    ++cnt;
  }
}

VertexPtr RandomTree::getNearestVertex(const Eigen::VectorXd& rand, const VertexPtr& root)
{
  VertexPtr v = root;
  double d = (root->getX() - rand).norm();

  for(unsigned int i = 0; i < root->getChild().size(); ++i)
  {
    double tmp = (root->getChild()[i]->getX() - rand).norm();
    if(tmp < d)
    {
      d = tmp;
      v = root->getChild()[i];
    }

    VertexPtr candidate = this->getNearestVertex(rand, root->getChild()[i]);
    if(candidate.get() == root->getChild()[i].get())
    {
      continue;
    }

    tmp = (candidate->getX() - rand).norm();
    if(tmp < d)
    {
      d = tmp;
      v = candidate;
    }
  }

  return v;
}

VertexPtr RandomTree::getNewVertex(const Eigen::VectorXd& rand, const VertexPtr& nearest)
{
  VertexPtr v = VertexPtr(new Vertex());
  double norm = (nearest->getX() - rand).norm();

  v->setX(nearest->getX() + param_->rho * (1.0 / norm) * (rand - nearest->getX()));
  return v;
}

bool RandomTree::reachedToGoal(const Eigen::VectorXd& new_x, const Eigen::VectorXd& dst_x)
{
  if((new_x - dst_x).norm() < param_->error_thresh)
  {
    return true;
  }

  return false;
}

#ifdef ENABLE_VISUALIZATION
void RandomTree::publish(const VertexPtr& parent, const VertexPtr& child)
{
  ahl_rrt::Edge3D msg;

  msg.x_parent = parent->getX().coeff(0);
  msg.y_parent = parent->getX().coeff(1);
  msg.z_parent = parent->getX().coeff(2);
  msg.x_child  = child->getX().coeff(0);
  msg.y_child  = child->getX().coeff(1);
  msg.z_child  = child->getX().coeff(2);

  while(ros::ok())
  {
    if(pub_.getNumSubscribers() > 0)
      break;

    ros::Duration(0.1).sleep();
  }

  pub_.publish(msg);
}

#endif
