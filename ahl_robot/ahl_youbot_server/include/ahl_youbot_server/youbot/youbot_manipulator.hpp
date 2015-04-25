#ifndef __AHL_YOUBOT_SERVER_YOUBOT_MANIPULATOR_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_MANIPULATOR_HPP

#include <vector>
#include <boost/shared_ptr.hpp>

namespace ahl_youbot
{

  class YouBotManipulator
  {
  public:
    virtual ~YouBotManipulator() {}

    // arm
    virtual void doJointCommutation() = 0;
    virtual void calibrateManipulator(const bool force_calibration = false) = 0;
    virtual int getNumberJoints() = 0;

    virtual void setAngle(const std::vector<double>& q) = 0;
    virtual void setVelocity(const std::vector<double>& dq) = 0;
    virtual void setTorque(const std::vector<double>& tau) = 0;

    virtual void getAngle(std::vector<double>& q) = 0;
    virtual void getVelocity(std::vector<double>& dq) = 0;
    virtual void setTorque(std::vector<double>& tau) = 0;

    // gripper
    virtual void calibrateGripper(const bool force_calibration = false) = 0;
    virtual void open() = 0;
    virtual void close() = 0;
    virtual void setSpace(double space) = 0;
    virtual void getSpace(double& space) = 0;
  };

  typedef boost::shared_ptr<YouBotManipulator> AHLYouBotManipulatorPtr;
}

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_MANIPULATOR_HPP */
