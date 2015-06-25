#include "ahl_robot_controller/common/effective_mass_matrix3d.hpp"

using namespace ahl_ctrl;

void EffectiveMassMatrix3d::compute(const Eigen::Matrix3d& lambda_inv, Eigen::Matrix3d& lambda, double thresh)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(lambda_inv, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Matrix3d S = svd.singularValues().asDiagonal();
  for(unsigned int i = 0; i < 3; ++i)
  {
    if(S.coeff(i, i) < thresh)
    {
      S.coeffRef(i, i) = thresh;
    }
  }

  lambda = svd.matrixV() * S.inverse() * svd.matrixU().transpose();
}
