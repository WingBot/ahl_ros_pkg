#ifndef __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ahl_robot/robot/mobility.hpp>
#include "ahl_robot_controller/exception.hpp"

namespace ahl_ctrl
{

  class MobilityController
  {
  public:
    virtual ~MobilityController() {}

    virtual void computeBaseVelocityFromTorque(
      const Eigen::MatrixXd& M, const Eigen::VectorXd& tau, Eigen::VectorXd& v_base)
    {
      throw ahl_ctrl::Exception("MobilityController::computeBaseVelocity", "This virtual function is not implemented.");
    }

    virtual void computeWheelVelocityFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& v_wheel)
    {
      throw ahl_ctrl::Exception("MobilityController::computeWheelVelocity", "This virtual function is not implemented.");
    }

    virtual void computeWheelTorqueFromBaseVelocity(
      const Eigen::VectorXd& v_base, Eigen::VectorXd& tau_wheel)
    {
      throw ahl_ctrl::Exception("MobilityController::computeWheelTorque", "This virtual function is not implemented.");
    }
  };

  typedef boost::shared_ptr<MobilityController> MobilityControllerPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_MOBILITY_CONTROLLER_HPP */
