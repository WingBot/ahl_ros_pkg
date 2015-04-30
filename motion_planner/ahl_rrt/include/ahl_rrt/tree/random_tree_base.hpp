#ifndef __AHL_RRT_RANDOM_TREE_BASE_HPP
#define __AHL_RRT_RANDOM_TREE_BASE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_rrt/param.hpp"
#include "ahl_rrt/tree/vertex.hpp"

namespace ahl_rrt
{

  class RandomTreeBase
  {
  public:
    virtual void init(const ParamPtr& param, const Eigen::VectorXd& init_x) = 0;
    virtual void build(const Eigen::VectorXd& dst_x) = 0;
  };

  typedef boost::shared_ptr<RandomTreeBase> RandomTreeBasePtr;
}

#endif /* __AHL_RRT_RANDOM_TREE_BASE_HPP */
