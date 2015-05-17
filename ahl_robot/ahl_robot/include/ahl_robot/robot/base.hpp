#ifndef __AHL_ROBOT_BASE_HPP
#define __AHL_ROBOT_BASE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot/robot/link.hpp"

namespace ahl_robot
{

  class Base
  {
  public:
    LinkPtr link;

    Eigen::Matrix4d T_org; // initial transformation matrix w.r.t world
    Eigen::Matrix4d T; // transformation matrix w.r.t world

    Eigen::Vector3d vd; // desired velocity
    Eigen::Vector3d wd; // desired angular velocity w.r.t T

    Eigen::Vector3d v; // desired velocity
    Eigen::Vector3d w; // desired angular velocity w.r.t T
  };

  typedef boost::shared_ptr<Base> BasePtr;
}

#endif /* __AHL_ROBOT_BASE_HPP */
