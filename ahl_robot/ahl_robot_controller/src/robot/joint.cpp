#include <cmath>
#include <climits>
#include "ahl_robot_controller/robot/joint.hpp"

using namespace ahl_robot;

Joint::Joint()
  : is_revolute(true), gear_ratio(1.0), I(Eigen::MatrixXd::Zero(I.rows(), I.cols())), M(0.0),
    q(0.0), q_min(-M_PI), q_max(M_PI),
    dq(0.0), dq_max(std::numeric_limits<double>::max()), ddq(0.0),
    tau(0.0), tau_max(std::numeric_limits<double>::max()),
    locked(false)
{
  org = Eigen::Matrix4d::Identity();
  axis.coeffRef(0) = 0.0;
  axis.coeffRef(1) = 0.0;
  axis.coeffRef(2) = 1.0;
}

void Joint::print()
{
  std::cout << "<< Joint name : " << name << " >>\n";

  if(is_revolute)
    std::cout << "type : revolute\n";
  else
    std::cout << "type : prismatic\n";

  std::cout << "Origin : " << std::endl << org << "\n"
            << "Inertia Matrix : " << std::endl << I << "\n"
            << "Mass : " << M << "\n"
            << "Gear ratio : " << gear_ratio << "\n"
            << "q : " << q << ", q_min : " << q_min << ", q_max : " << q_max << "\n"
            << "dq : " << dq << ", dq_max : " << dq_max << "\n"
            << "ddq : " << ddq << "\n"
            << "tau : " << tau << ", tau_max : " << tau_max << "\n";

  if(locked)
    std::cout << "locked : true\n";
  else
    std::cout << "locked : false\n";

  std::cout << "parent : " << parent_link->name << "\n"
            << "child : " << link->name << "\n\n";
}
