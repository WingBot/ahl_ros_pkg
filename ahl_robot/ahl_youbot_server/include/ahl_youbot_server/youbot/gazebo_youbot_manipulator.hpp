#ifndef __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_MANIPULATOR_HPP
#define __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_MANIPULATOR_HPP

#include "ahl_youbot_server/youbot/youbot_manipulator.hpp"

namespace ahl_youbot
{

  class GazeboYouBotManipulator : public YouBotManipulator
  {
  public:
    GazeboYouBotManipulator();

    // arm
    virtual void doJointCommutation();
    virtual void calibrateManipulator(const bool force_calibration = false);
    virtual int getNumberJoints();

    virtual void setAngle(const std::vector<double>& q);
    virtual void setVelocity(const std::vector<double>& dq);
    virtual void setTorque(const std::vector<double>& tau);

    virtual void getAngle(std::vector<double>& q);
    virtual void getVelocity(std::vector<double>& dq);
    virtual void setTorque(std::vector<double>& tau);

    // gripper
    virtual void calibrateGripper(const bool force_calibration = false);
    virtual void open();
    virtual void close();
    virtual void setSpace(double space);
    virtual void getSpace(double& space);

  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_MANIPULATOR_HPP */
