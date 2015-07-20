#ifndef __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP
#define __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP

#include <Eigen/Dense>
#include "ahl_digital_filter/differentiator.hpp"
#include "ahl_digital_filter/pseudo_differentiator.hpp"

namespace ahl_filter
{

  class PseudoDifferentiator : public Differentiator
  {
  public:
    PseudoDifferentiator(double period, double cutoff_freq);

    virtual void init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq);
    virtual void apply(const Eigen::VectorXd& q);
    virtual void copyDerivativeValueTo(Eigen::VectorXd& dq)
    {
      dq = dq_;
    }

  private:
    Eigen::VectorXd q_;
    Eigen::VectorXd dq_;
    Eigen::VectorXd pre_q_;
    Eigen::VectorXd pre_dq_;
    double period_;
    double T_;

    double coeff1_;
    double coeff2_;
  };

}

#endif /* __AHL_DIGITAL_FILTER_PSEUDO_DIFFERENTIATOR_HPP */
