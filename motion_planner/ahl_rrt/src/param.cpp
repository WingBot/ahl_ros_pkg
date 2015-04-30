#include "ahl_rrt/param.hpp"

using namespace ahl_rrt;

Param::Param()
  : max_iterations(10000), error_thresh(1.0), rho(1.0), dt(0.1)
{
  max.resize(3);
  min.resize(max.rows());

  max.coeffRef(0) = -50.0;
  max.coeffRef(1) = -50.0;
  max.coeffRef(2) = -50.0;

  min.coeffRef(0) = 50.0;
  min.coeffRef(1) = 50.0;
  min.coeffRef(2) = 50.0;
}
