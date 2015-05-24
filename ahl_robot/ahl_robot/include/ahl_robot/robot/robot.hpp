#ifndef __AHL_ROBOT_ROBOT_HPP
#define __AHL_ROBOT_ROBOT_HPP

#include "ahl_robot/definition.hpp"
#include "ahl_robot/robot/manipulator.hpp"

namespace ahl_robot
{
  typedef std::map<std::string, ManipulatorPtr, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, ManipulatorPtr> > > MapManipulatorPtr;

  class Robot
  {
  public:
    Robot(const std::string& robot_name)
      : name_(robot_name), world_(frame::WORLD)
    {
    }

    void setPosition(const Eigen::Vector3d& p)
    {
      p_ = p;
    }

    void setOrientation(const Eigen::Quaternion<double>& q)
    {
      q_ = q;
    }

    void setWorldFrame(const std::string& world)
    {
      world_ = world;
    }

    void add(const ManipulatorPtr& mnp)
    {
      mnp_[mnp->name] = mnp;
      mnp_name_.push_back(mnp->name);
    }

    const std::string& getName() const
    {
      return name_;
    }

    const Eigen::Vector3d& getPosition() const
    {
      return p_;
    }

    const Eigen::Quaternion<double>& getOrientation() const
    {
      return q_;
    }

    const std::string& getWorldFrame() const
    {
      return world_;
    }

    const ManipulatorPtr& getManipulator(const std::string& name)
    {
      return mnp_[name];
    }

    const std::vector<std::string>& getManipulatorName() const
    {
      return mnp_name_;
    }

  private:
    std::string name_;
    Eigen::Vector3d p_;
    Eigen::Quaternion<double> q_;
    MapManipulatorPtr mnp_;
    std::vector<std::string> mnp_name_;
    std::string world_;
  };

  typedef boost::shared_ptr<Robot> RobotPtr;
}

#endif /* __AHL_ROBOT_ROBOT_HPP */
