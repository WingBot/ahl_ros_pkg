#include "ahl_rrt/rrt.hpp"
#include "ahl_rrt/exception.hpp"
#include "ahl_rrt/tree/random_tree.hpp"

using namespace ahl_rrt;

RRT::RRT()
  : initialized_(false), generated_tree_(false)
{
  tree_ = RandomTreeBasePtr(new RandomTree());
}

void RRT::init(const ParamPtr& param, const Eigen::VectorXd& init_x)
{
  tree_->init(param, init_x);

  initialized_ = true;
}

void RRT::buildTree(const Eigen::VectorXd& dst_x)
{
  if(!initialized_)
    throw ahl_rrt::Exception("RRT::generateTree", "ahl_rrt::RRT is not initialized.");

  tree_->build(dst_x);
  generated_tree_ = true;
}
