#ifndef __AHL_ROBOT_CONTROLLER_MOBILE_BASE_HPP
#define __AHL_ROBOT_CONTROLLER_MOBILE_BASE_HPP

#include <vector>
#include "ahl_robot_controller/robot/link.hpp"
#include "ahl_robot_controller/robot/joint.hpp"

namespace ahl_robot
{

  class MobileBase
  {
  public:
    virtual ~MobileBase() {}

  private:

  };

  typedef boost::shared_ptr<MobileBase> MobileBasePtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_MOBILE_BASE_HPP */
