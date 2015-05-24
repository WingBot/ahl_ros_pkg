#ifndef __AHL_ROBOT_DEFINITION_HPP
#define __AHL_ROBOT_DEFINITION_HPP

#include <string>

namespace ahl_robot
{
  namespace joint
  {
    static const std::string REVOLUTE_X        = "revolute_x";
    static const std::string REVOLUTE_Y        = "revolute_y";
    static const std::string REVOLUTE_Z        = "revolute_z";
    static const std::string PRISMATIC_X       = "prismatic_x";
    static const std::string PRISMATIC_Y       = "prismatic_y";
    static const std::string PRISMATIC_Z       = "prismatic_z";
    static const std::string FIXED             = "fixed";
  }

  namespace frame
  {
    static const std::string WORLD = "map";
  }
}

#endif /* __AHL_ROBOT_DEFINITION_HPP */
