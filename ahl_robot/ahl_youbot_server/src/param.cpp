#include "ahl_youbot_server/param.hpp"

using namespace ahl_youbot;

Param::Param()
{
  g_ << 0.0, 0.0, -9.80665;
  f_ = boost::bind(&Param::update, this, _1, _2);
  server_.setCallback(f_);
}

void Param::update(ahl_youbot_server::ParamConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  Eigen::VectorXd Kp(8);
  Kp << config.kp1, config.kp2, config.kp3, config.kp4, config.kp5, config.kp6, config.kp7, config.kp8;
  Kp_ = Kp.asDiagonal();

  Eigen::VectorXd Kv(8);
  Kv << config.kv1, config.kv2, config.kv3, config.kv4, config.kv5, config.kv6, config.kv7, config.kv8;
  Kv_ = Kv.asDiagonal();
}
