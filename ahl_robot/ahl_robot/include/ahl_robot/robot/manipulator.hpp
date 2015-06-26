/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

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
