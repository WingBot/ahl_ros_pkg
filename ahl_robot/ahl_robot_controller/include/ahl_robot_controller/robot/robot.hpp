#ifndef __AHL_ROBOT_CONTROLLER_ROBOT_HPP
#define __AHL_ROBOT_CONTROLLER_ROBOT_HPP

#include <string>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include "ahl_robot_controller/robot/joint.hpp"
#include "ahl_robot_controller/robot/link.hpp"
#include "ahl_robot_controller/robot/manipulator.hpp"
#include "ahl_robot_controller/robot/mobile_base.hpp"

namespace ahl_robot
{

  class Robot
  {
  public:
    Robot();
    ~Robot() {}

    void fix();
    void unfix();
    void addLink(const LinkPtr& link);
    void addJoint(const JointPtr& joint);

    void connectJointWithLink(const std::string& joint, const std::string& link);
    void connectJointWithParentLink(const std::string& joint, const std::string& link);

    void setBaseFrame(const std::string& name);
    void setOperatedFrame(const std::string& name, const std::string& parent, const Eigen::Vector3d& xyz, const Eigen::Vector3d& rpy);

    void setup();

    const Joints& getJoints() const;
    const Links& getLinks() const;
    const LinkPtr& getBaseLink() const;
    const ManipulatorPtr& getManipulator() const;
    const MobileBasePtr& getMobileBase() const;

    void print();
    void printNameList();

  private:
    void setupManipulator();
    void setupMobileBase();

    bool is_static_;
    bool initialized_;

    Joints joints_;
    Links links_;

    std::string base_frame_;
    std::string operated_frame_;

    ManipulatorPtr manipulator_;
    MobileBasePtr mobile_base_;
  };

  typedef boost::shared_ptr<Robot> RobotPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_ROBOT_HPP */
