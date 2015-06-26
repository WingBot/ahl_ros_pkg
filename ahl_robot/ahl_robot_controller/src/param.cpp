#include "ahl_robot_controller/param.hpp"

using namespace ahl_ctrl;

Param::Param()
  : kp_(0.0), kv_(0.0), kv_damp_(0.0),
    kp_limit_(0.0), kv_limit_(0.0)
{
  g_ << 0.0, 0.0, -9.80665;
  f_ = boost::bind(&Param::update, this, _1, _2);
  server_.setCallback(f_);
}

void Param::update(ahl_robot_controller::ParamConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  kp_ = config.kp;
  kv_ = config.kv;
  kv_damp_ = config.kv_damp;
  kp_limit_ = config.kp_limit;
  kv_limit_ = config.kv_limit;
  g_ << config.gx, config.gy, config.gz;
}
