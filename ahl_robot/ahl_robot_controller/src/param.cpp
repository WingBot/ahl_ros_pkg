#include "ahl_robot_controller/param.hpp"

using namespace ahl_ctrl;

Param::Param()
  : kp_joint(0.0), kv_joint(0.0),
    kp_task(0.0), kv_task(0.0),
    kp_limit(0.0), kv_limit(0.0)
{
  g << 0.0, 0.0, -9.80665;
}
