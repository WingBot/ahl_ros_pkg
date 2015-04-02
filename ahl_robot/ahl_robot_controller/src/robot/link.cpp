#include "ahl_robot_controller/robot/link.hpp"

using namespace ahl_robot;

Link::Link()
  : I(Eigen::MatrixXd::Zero(I.rows(), I.cols())),
    org(Eigen::MatrixXd::Zero(org.rows(), org.cols())),
    com(Eigen::MatrixXd::Zero(com.rows(), com.cols())),
    M(0.0)
{
}

void Link::print()
{
  std::cout << "<< Link name : " << name << " >>\n"
            << "Inertia matrix : " << std::endl << I << "\n"
            << "Origin : " << std::endl << org << "\n"
            << "Center of Mass : " << std::endl << com << "\n"
            << "Mass : " << M << "\n\n";
}
