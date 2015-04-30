#ifndef __AHL_RRT_PARAM_HPP
#define __AHL_RRT_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_rrt
{

  class Param
  {
  public:
    Param();

    unsigned long max_iterations;
    double error_thresh;
    Eigen::VectorXd max;
    Eigen::VectorXd min;
    double rho;
    double dt;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_RRT_PARAM_HPP */
