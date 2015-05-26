#ifndef __AHL_ROBOT_MANIPULATOR_HPP
#define __AHL_ROBOT_MANIPULATOR_HPP

#include <vector>
#include <Eigen/StdVector>
#include "ahl_robot/robot/link.hpp"

namespace ahl_robot
{
  typedef std::vector< LinkPtr, Eigen::aligned_allocator<LinkPtr> > VectorLinkPtr;
  typedef std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;
  typedef std::vector<Eigen::Matrix3d> VectorMatrix3d;
  typedef std::vector<Eigen::Vector3d> VectorVector3d;

  class Manipulator
  {
  public:
    Manipulator();
    void print();
    void init(unsigned int dof, const Eigen::VectorXd& init_q);
    void update(const Eigen::VectorXd& q_msr);
    void computeJacobian(const std::string& name);

    std::string name;
    VectorLinkPtr link;

    unsigned int dof;

    Eigen::MatrixXd J0; // Basic jacobian associated with end effector
    Eigen::MatrixXd J; // Jacobian associated with end effector
    Eigen::VectorXd q; // Generalized coordinates
    Eigen::VectorXd pre_q; // Generalized coordinates
    Eigen::VectorXd dq; // Velocity of generalized coordinates

    Eigen::Vector3d xp;  // xyz position
    Eigen::Vector3d pre_xp;  // xyz position
    Eigen::Vector3d dxp; // xyz velocity
    Eigen::Quaternion<double> xr;  // Quaternion
    Eigen::Vector3d w; // Angular velocity

    VectorMatrix4d T; // Relative transformation matrix associated with each link frame
  private:
    void computeForwardKinematics();
    void computeRbiPin(); // Should be called after updating xp
    void computeJacobian(int idx);

    double time_;
    double pre_time_;
    VectorMatrix3d Rbi_; // Rotation matrix of i-th link w.r.t base
    VectorMatrix4d T_abs_; // Transformation matrix of i-th link w.r.t base
    VectorVector3d Pin_; // End-effector position w.r.t i-th link w.r.t link
  };

  typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
}

#endif /* __AHL_ROBOT_MANIPULATOR_HPP */
