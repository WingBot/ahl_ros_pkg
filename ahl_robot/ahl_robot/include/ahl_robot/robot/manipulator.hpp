#ifndef __AHL_ROBOT_MANIPULATOR_HPP
#define __AHL_ROBOT_MANIPULATOR_HPP

#include <map>
#include <vector>
#include <Eigen/StdVector>
#include <ahl_digital_filter/differentiator.hpp>
#include "ahl_robot/robot/link.hpp"

namespace ahl_robot
{
  typedef std::vector< LinkPtr, Eigen::aligned_allocator<LinkPtr> > VectorLinkPtr;
  typedef std::vector< Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorMatrix4d;
  typedef std::vector<Eigen::Matrix3d> VectorMatrix3d;
  typedef std::vector<Eigen::MatrixXd> VectorMatrixXd;
  typedef std::vector<Eigen::Vector3d> VectorVector3d;
  typedef std::map<std::string, Eigen::MatrixXd, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::MatrixXd> > > MapMatrixXd;

  class Manipulator
  {
  public:
    Manipulator();
    void init(unsigned int dof, const Eigen::VectorXd& init_q);
    void update(const Eigen::VectorXd& q_msr);
    void computeBasicJacobian();
    void computeMassMatrix();
    bool reached(const Eigen::VectorXd& qd, double threshold);
    void print();

    //void computeBasicJacobian(const std::string& name);

    std::string name;
    VectorLinkPtr link;
    std::map<std::string, int> name_to_idx;

    unsigned int dof;

    //Eigen::MatrixXd J0; // Basic jacobian associated with end effector
    VectorMatrixXd J0; // Basic jacobian associated with link jacobian
    Eigen::MatrixXd M; // Mass matrix
    Eigen::MatrixXd M_inv;

    Eigen::VectorXd q; // Generalized coordinates
    Eigen::VectorXd pre_q; // Generalized coordinates
    Eigen::VectorXd dq; // Velocity of generalized coordinates

    Eigen::Vector3d xp;  // xyz position
    Eigen::Quaternion<double> xr;  // Quaternion

    VectorMatrix4d T; // Relative transformation matrix associated with each link frame
    VectorMatrix4d T_abs; // Transformation matrix of i-th link w.r.t base

  private:
    void computeForwardKinematics();
    void computeTabs(); // Should be called after updating xp
    void computeCabs(); // Should be called after updating xp

    void computeBasicJacobian(int idx, Eigen::MatrixXd& J);
    void computeVelocity();

    double time_;
    double pre_time_;
    VectorMatrix4d C_abs_; // Transformation matrix of i-th center of mass w.r.t base
    VectorVector3d Pin_; // End-effector position w.r.t i-th link w.r.t link

    ahl_filter::DifferentiatorPtr differentiator_;
    bool updated_joint_;
  };

  typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
}

#endif /* __AHL_ROBOT_MANIPULATOR_HPP */
