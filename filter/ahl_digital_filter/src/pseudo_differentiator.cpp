#include "ahl_digital_filter/exception.hpp"
#include "ahl_digital_filter/pseudo_differentiator.hpp"

using namespace ahl_filter;

PseudoDifferentiator::PseudoDifferentiator(double period, double cutoff_freq)
  : period_(period)
{
  if(period_ <= 0.0)
  {
    std::stringstream msg;
    msg << "Period should not be smaller than zero.";
    throw ahl_filter::Exception("PseudoDifferentiator::PseudoDifferentiator", msg.str());
  }

  if(cutoff_freq <= 0.0)
  {
    std::stringstream msg;
    msg << "Cutoff frequency should not be smaller than zero.";
    throw ahl_filter::Exception("PseudoDifferentiator::PseudoDifferentiator", msg.str());
  }

  T_ = 1.0 / (2.0 * M_PI * cutoff_freq);

  coeff1_ = sqrt((1.0 + exp(-2.0 * period_ / T_) - 2.0 * exp(-period_ / T_) * cos(period_)) / (2.0 * (1.0 - cos(period_))));
  coeff2_ = exp(-period_ / T_);
}

void PseudoDifferentiator::init(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
{
  if(q.rows() != dq.rows())
  {
    std::stringstream msg;
    msg << "q.rows() != dq.rows()" << std::endl
        << "  q.rows  : " << q.rows() << std::endl
        << "  dq.rows : " << dq.rows();

    throw ahl_filter::Exception("PseudoDifferentiator::init", msg.str());
  }

  q_ = pre_q_ = q;
  dq_ = pre_dq_ = dq;
}

void PseudoDifferentiator::apply(const Eigen::VectorXd& q)
{
  dq_ = coeff1_ * (q - pre_q_) + coeff2_ * pre_dq_;

  pre_q_ = q;
  pre_dq_ = dq_;
}
