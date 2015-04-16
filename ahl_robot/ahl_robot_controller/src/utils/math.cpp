#include "ahl_robot_controller/utils/math.hpp"

namespace ahl_robot
{
  namespace utils
  {

    int sign(double src1, double src2)
    {
      if(src1 - src2 >= 0.0)
        return 1;
      else
        return -1;
    }

    void convertRPYToRotationMatrix(double r, double p, double y, Eigen::Matrix3d& dst)
    {
      static const Eigen::Vector3d x_axis(1.0, 0.0, 0.0);
      static const Eigen::Vector3d y_axis(0.0, 1.0, 0.0);
      static const Eigen::Vector3d z_axis(0.0, 0.0, 1.0);

      dst = Eigen::AngleAxisd(y, z_axis) * Eigen::AngleAxisd(p, y_axis) * Eigen::AngleAxisd(r, x_axis);
    }

    Eigen::MatrixXd& getLastRowOfTransformationMatrix()
    {
      static Eigen::MatrixXd last_row = Eigen::Matrix4d::Identity().block(3, 0, 1, 4);
      return last_row;
    }

    void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst)
    {
      Eigen::Matrix3d R_transpose = src.block(0, 0, 3, 3).transpose();
      dst.block(0, 0, 3, 3) = R_transpose;
      dst.block(0, 3, 3, 1) = -R_transpose * src.block(0, 3, 3, 1);
      dst.block(3, 0, 1, 4) = utils::getLastRowOfTransformationMatrix();
    }

  }
}
