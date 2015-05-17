#ifndef __AHL_ROBOT_CONTROLLER_MANIPULATOR_HPP
#define __AHL_ROBOT_CONTROLLER_MANIPULATOR_HPP

#include <vector>
#include "ahl_robot_controller/robot/link.hpp"
#include "ahl_robot_controller/robot/joint.hpp"

namespace ahl_robot
{

  class Manipulator
  {
  public:
    void addLink(const LinkPtr& link);
    void addJoint(const JointPtr& joint);
    void setup();

    void print();

  private:
    std::vector<LinkPtr> links_;
    std::vector<JointPtr> joints_;
  };

  typedef boost::shared_ptr<Manipulator> ManipulatorPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_MANIPULATOR_HPP */
