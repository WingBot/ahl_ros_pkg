#ifndef __AHL_RRT_RRT_BASE_HPP
#define __AHL_RRT_RRT_BASE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_rrt/param.hpp"
#include "ahl_rrt/tree/random_tree_base.hpp"

namespace ahl_rrt
{

  class RRTBase
  {
  public:
    virtual ~RRTBase() {}

    virtual void init(const ParamPtr& param, const Eigen::VectorXd& init_x) = 0;
    virtual void buildTree(const Eigen::VectorXd& dst_x) = 0;
    virtual const RandomTreeBasePtr& getTree() const = 0;
    virtual const std::vector<Eigen::VectorXd>& getPath() const = 0;
  };

  typedef boost::shared_ptr<RRTBase> RRTBasePtr;
}

#endif /* __AHL_RRT_RRT_BASE_HPP */
