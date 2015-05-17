#include "ahl_robot/utils/math.hpp"

namespace ahl_robot
{
  namespace math
  {
    void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst)
    {
      dst = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d R_trans = src.block(0, 0, 3, 3).transpose();
      dst.block(0, 0, 3, 3) = R_trans;
      dst.block(0, 3, 3, 1) = -R_trans * src.block(0, 3, 3, 1);
    }

    void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat)
    {
      double sin_a = sin(rpy[2]);
      double cos_a = cos(rpy[2]);
      double sin_b = sin(rpy[1]);
      double cos_b = cos(rpy[1]);
      double sin_g = sin(rpy[0]);
      double cos_g = cos(rpy[0]);

      mat.coeffRef(0, 0) = cos_a * cos_b;
      mat.coeffRef(0, 1) = cos_a * sin_b * sin_g - sin_a * cos_g;
      mat.coeffRef(0, 2) = cos_a * sin_b * cos_g + sin_a * sin_g;
      mat.coeffRef(1, 0) = sin_a * cos_b;
      mat.coeffRef(1, 1) = sin_a * sin_b * sin_g + cos_a * cos_g;
      mat.coeffRef(1, 2) = sin_a * sin_b * cos_g - cos_a * sin_g;
      mat.coeffRef(2, 0) = -sin_b;
      mat.coeffRef(2, 1) = cos_b * sin_g;
      mat.coeffRef(2, 2) = cos_b * cos_g;
    }

    void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat)
    {
      double sin_a = sin(rpy.coeff(2));
      double cos_a = cos(rpy.coeff(2));
      double sin_b = sin(rpy.coeff(1));
      double cos_b = cos(rpy.coeff(1));
      double sin_g = sin(rpy.coeff(0));
      double cos_g = cos(rpy.coeff(0));

      mat.coeffRef(0, 0) = cos_a * cos_b;
      mat.coeffRef(0, 1) = cos_a * sin_b * sin_g - sin_a * cos_g;
      mat.coeffRef(0, 2) = cos_a * sin_b * cos_g + sin_a * sin_g;
      mat.coeffRef(1, 0) = sin_a * cos_b;
      mat.coeffRef(1, 1) = sin_a * sin_b * sin_g + cos_a * cos_g;
      mat.coeffRef(1, 2) = sin_a * sin_b * cos_g - cos_a * sin_g;
      mat.coeffRef(2, 0) = -sin_b;
      mat.coeffRef(2, 1) = cos_b * sin_g;
      mat.coeffRef(2, 2) = cos_b * cos_g;
    }
  }
}
