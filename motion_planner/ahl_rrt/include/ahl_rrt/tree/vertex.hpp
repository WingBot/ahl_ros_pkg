#ifndef __AHL_RRT_VERTEX_HPP
#define __AHL_RRT_VERTEX_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <Eigen/Dense>

namespace ahl_rrt
{

  class Vertex;
  typedef boost::shared_ptr<Vertex> VertexPtr;

  class Vertex : public boost::enable_shared_from_this<Vertex>
  {
  public:
    void addChild(VertexPtr& child);
    void setParent(const VertexPtr& parent);

    void setX(const Eigen::VectorXd& x)
    {
      x_ = x;
    }

    const Eigen::VectorXd& getX() const
    {
      return x_;
    }

    const VertexPtr& getParent() const
    {
      return parent_;
    }

    const std::vector<VertexPtr> getChild() const
    {
      return child_;
    }

  private:
    Eigen::VectorXd x_;
    VertexPtr parent_;
    std::vector<VertexPtr> child_;
  };

}

#endif /* __AHL_RRT_VERTEX_HPP */
