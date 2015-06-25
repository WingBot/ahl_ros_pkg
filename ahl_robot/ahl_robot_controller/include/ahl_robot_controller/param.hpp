#ifndef __AHL_ROBOT_CONTROLLER_PARAM_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_ctrl
{

  class Param
  {
  public:
    Param();
    double kp_joint;
    double kv_joint;
    double kp_task;
    double kv_task;
    double kp_limit;
    double kv_limit;
    Eigen::Vector3d g;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_HPP */
