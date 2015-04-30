#ifndef __AHL_RRT_RRT_HPP
#define __AHL_RRT_RRT_HPP

#include "ahl_rrt/rrt_base.hpp"

namespace ahl_rrt
{

  class RRT : public RRTBase
  {
  public:
    RRT();

    virtual void init(const ParamPtr& param, const Eigen::VectorXd& init_x);
    virtual void buildTree(const Eigen::VectorXd& dst_x);

    virtual const RandomTreeBasePtr& getTree() const
    {
      return tree_;
    }

    virtual const std::vector<Eigen::VectorXd>& getPath() const
    {
      return path_;
    }

  private:
    bool initialized_;
    bool generated_tree_;

    RandomTreeBasePtr tree_;
    std::vector<Eigen::VectorXd> path_;
  };

}

#endif /* __AHL_RRT_RRT_HPP */
