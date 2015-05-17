#ifndef __AHL_ROBOT_MANIPULATOR_HPP
#define __AHL_ROBOT_MANIPULATOR_HPP

#include <boost/shared_ptr.hpp>
#include "ahl_robot/robot/link.hpp"

namespace ahl_robot
{

  class Manipulator
  {
  public:
    Manipulator()
      : name(""), op_name("")
    {
      T_op = Eigen::Matrix4d::Identity();
    }

    std::string name;
    std::vector<LinkPtr> link;
    std::string op_name; // operational point
    Eigen::Matrix4d T_op; // transformation matrix attached to operational point w.r.t specified frame through setOperationalPoint method.
  };

  typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
}

#endif /* __AHL_ROBOT_MANIPULATOR_HPP */
