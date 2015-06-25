#ifndef __AHL_ROBOT_ROBOT_HPP
#define __AHL_ROBOT_ROBOT_HPP

#include <map>
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

    void update(const std::string& mnp_name, const Eigen::VectorXd& q);
    void computeBasicJacobian(const std::string& mnp_name);
    void computeMassMatrix(const std::string& mnp_name);
    bool reached(const std::string& mnp_name, const Eigen::VectorXd& qd, double threshold);

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

    void add(const ManipulatorPtr& mnp);

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

    const Eigen::MatrixXd& getBasicJacobian(const std::string& mnp_name);
    const Eigen::MatrixXd& getBasicJacobian(const std::string& mnp_name, const std::string& link_name);

    const Eigen::VectorXd& getJointPosition(const std::string& mnp_name);
    const Eigen::VectorXd& getJointVelocity(const std::string& mnp_name);
    const Eigen::MatrixXd& getMassMatrix(const std::string& mnp_name);
    const Eigen::MatrixXd& getMassMatrixInv(const std::string& mnp_name);
    unsigned int getDOF(const std::string& mnp_name);

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
