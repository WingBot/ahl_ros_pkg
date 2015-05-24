#ifndef __AHL_ROBOT_MATH_HPP
#define __AHL_ROBOT_MATH_HPP

#include <vector>
#include <Eigen/Dense>

namespace ahl_robot
{
  namespace math
  {
    void calculateInverseTransformationMatrix(const Eigen::Matrix4d& src, Eigen::Matrix4d& dst);
    void rpyToRotationMatrix(const std::vector<double>& rpy, Eigen::Matrix3d& mat);
    void rpyToRotationMatrix(const Eigen::Vector3d& rpy, Eigen::Matrix3d& mat);
    void rpyToQuaternion(const Eigen::Vector3d& rpy, Eigen::Quaternion<double>& q);
    void xyzrpyToTransformationMatrix(const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy, Eigen::Matrix4d& T);
  }
}

#endif /* __AHL_ROBOT_MATH_HPP */
