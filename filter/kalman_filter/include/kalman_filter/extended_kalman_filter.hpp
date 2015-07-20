#ifndef __KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
#define __KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

#include "kalman_filter/normal_distribution.hpp"

namespace kf
{

  class ExtendedKalmanFilter
  {
  public:
    void setRandomVariables(const NormalDistributionPtr& state,
                            const NormalDistributionPtr& uncertainty,
                            const NormalDistributionPtr& msr_noise);
    void setModel(

  private:


    NormalDistributionPtr state_;
    NormalDistributionPtr predicted_state_;
    NormalDistributionPtr uncertainty_;
    NormalDistributionPtr msr_noise_;
  };

  typedef boost::shared_ptr<ExtendedKalmanFilter> ExtendedKalmanFilterPtr;
}

#endif /* __KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP */
