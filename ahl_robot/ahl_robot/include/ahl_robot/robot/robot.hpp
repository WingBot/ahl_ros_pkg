#ifndef __AHL_ROBOT_ROBOT_HPP
#define __AHL_ROBOT_ROBOT_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include "ahl_robot/robot/link.hpp"
#include "ahl_robot/robot/manipulator.hpp"
#include "ahl_robot/tf/tf_link.hpp"
#include "ahl_robot/tf/tf_joint.hpp"

namespace ahl_robot
{
  //typedef std::vector<LinkPtr> Manipulator;
  //typedef boost::shared_ptr<Manipulator> ManipulatorPtr;

  class Robot
  {
  public:
    Robot(const std::string& robot_name, const std::string& base_link_name,
          const std::map<std::string, std::string>& mnp_name_to_ee_name)
      : name(robot_name), base_name(base_link_name), mnp_to_ee(mnp_name_to_ee_name)
    {
    }

    std::string name;
    std::string base_name;

    std::map<std::string, std::string> mnp_to_ee;
    std::map<std::string, ManipulatorPtr> mnp;

    TfJointMap tf_joint;
    TfLinkMap tf_link;
  };

  typedef boost::shared_ptr<Robot> RobotPtr;
}

#endif /* __AHL_ROBOT_ROBOT_HPP */
