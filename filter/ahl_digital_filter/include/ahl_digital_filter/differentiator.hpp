#ifndef __AHL_DIGITAL_FILTER_DIFFERENTIATOR_HPP
#define __AHL_DIGITAL_FILTER_DIFFERENTIATOR_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_filter
{

  class Differentiator
  {
  public:
    virtual ~Differentiator() {}
    virtual void init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq) = 0;
    virtual void apply(const Eigen::VectorXd& q) = 0;
    virtual void copyDerivativeValueTo(Eigen::VectorXd& dq) = 0;
  };

  typedef boost::shared_ptr<Differentiator> DifferentiatorPtr;
}

#endif /* __AHL_DIGITAL_FILTER_DIFFERENTIATOR_HPP */
