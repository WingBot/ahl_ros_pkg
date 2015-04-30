#ifndef __AHL_RRT_RANDOM_TREE_HPP
#define __AHL_RRT_RANDOM_TREE_HPP

#include <ros/ros.h>
#include "ahl_rrt/tree/random_tree_base.hpp"

#define ENABLE_VISUALIZATION

namespace ahl_rrt
{

  class RandomTree : public RandomTreeBase
  {
  public:
    virtual void init(const ParamPtr& param, const Eigen::VectorXd& init_x);
    virtual void build(const Eigen::VectorXd& dst_x);
  private:
    VertexPtr getNearestVertex(const Eigen::VectorXd& rand, const VertexPtr& root);
    VertexPtr getNewVertex(const Eigen::VectorXd& rand, const VertexPtr& nearest);
    bool reachedToGoal(const Eigen::VectorXd& new_x, const Eigen::VectorXd& dst_x);

    ParamPtr param_;
    VertexPtr root_;

#ifdef ENABLE_VISUALIZATION
  private:
    void publish(const VertexPtr& parent, const VertexPtr& child);
    ros::Publisher pub_;
#endif
  };

}

#endif /* __AHL_RRT_RANDOM_TREE_HPP */
