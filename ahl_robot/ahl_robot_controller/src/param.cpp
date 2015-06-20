#include "ahl_robot_controller/param.hpp"

using namespace ahl_ctrl;

Param::Param()
  : kp_joint_(0.0), kv_joint_(0.0), kp_task_(0.0), kv_task_(0.0)
{
  g_ << 0.0, 0.0, -9.80665;
}
