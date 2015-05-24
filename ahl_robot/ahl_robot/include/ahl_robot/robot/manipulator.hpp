#ifndef __AHL_ROBOT_MANIPULATOR_HPP
#define __AHL_ROBOT_MANIPULATOR_HPP

#include <vector>
#include <Eigen/StdVector>
#include "ahl_robot/robot/link.hpp"

namespace ahl_robot
{
  typedef std::vector< LinkPtr, Eigen::aligned_allocator<LinkPtr> > VectorLinkPtr;
  typedef std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;

  class Manipulator
  {
  public:
    Manipulator();
    void print();
    void computeFK();

    std::string name;
    VectorLinkPtr link;

    unsigned int dof;

    Eigen::MatrixXd M; // Mass matrix
    Eigen::MatrixXd M_inv;
    Eigen::MatrixXd J; // Jacobian
    Eigen::MatrixXd J_inv; // dynamics consistency
    Eigen::VectorXd q; // generalized coordinates
    Eigen::VectorXd dq; // velocity of generalized coordinates

    Eigen::MatrixXd Lambda_x; // Effective mass matrix for translation
    Eigen::MatrixXd Lambda_x_inv;
    Eigen::MatrixXd Lambda_w; // Effective mass matrix for rotation
    Eigen::MatrixXd Lambda_w_inv;
    Eigen::Vector3d xp;  // xyz position
    Eigen::Vector3d dxp; // xyz velocity
    Eigen::Quaternion<double> xr;  // quaternion
    Eigen::Vector3d dxr; // angular velocity
    Eigen::Matrix3d Rn; // End effector's rotation matrix

    VectorMatrix4d T; // Relative transformation matrix associated with each link frame
  };

  typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
}

#endif /* __AHL_ROBOT_MANIPULATOR_HPP */
