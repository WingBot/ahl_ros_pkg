#include <ros/ros.h>
#include "ahl_digital_filter/differentiator.hpp"
#include "ahl_digital_filter/pseudo_differentiator.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ahl_digital_filter_test");
  ros::NodeHandle nh;

  using namespace ahl_filter;
  Eigen::VectorXd q(1), qd(1);
  double period = 0.001;
  double cutoff = 30.0;

  DifferentiatorPtr differentiator = DifferentiatorPtr(
    new PseudoDifferentiator(period, cutoff));
  differentiator->init(q, qd);

  ros::Rate r(1.0 / period);
  double t = 0.0;
  double f = 1.0;

  Eigen::VectorXd pre_q = Eigen::VectorXd::Zero(1);

  while(ros::ok())
  {
    q = Eigen::VectorXd::Constant(1, sin(2.0 * M_PI * f * t));
    differentiator->apply(q);
    std::cout << "new qd : " << std::endl << qd << std::endl;
    std::cout << "answer2 : " << 1.0 / period * (q - pre_q) << std::endl;
    pre_q = q;
    t += period;
    r.sleep();
  }

  return 0;
}
