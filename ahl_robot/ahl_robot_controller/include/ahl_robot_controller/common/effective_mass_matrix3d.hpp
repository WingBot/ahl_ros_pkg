#ifndef __AHL_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP
#define __AHL_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP

#include <Eigen/Dense>

namespace ahl_ctrl
{

  class EffectiveMassMatrix3d
  {
  public:
    static void compute(const Eigen::Matrix3d& lambda_inv, Eigen::Matrix3d& lambda, double thresh = 0.0);
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_EFFECTIVE_MASS_MATRIX3D_HPP */
