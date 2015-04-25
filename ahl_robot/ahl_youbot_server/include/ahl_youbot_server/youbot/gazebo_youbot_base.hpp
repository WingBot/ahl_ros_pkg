#ifndef __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_BASE_HPP
#define __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_BASE_HPP

#include "ahl_youbot_server/youbot/youbot_base.hpp"

namespace ahl_youbot
{

  class GazeboYouBotBase : public YouBotBase
  {
  public:
    GazeboYouBotBase();

    virtual void doJointCommutation();
    virtual void setPosition(double x, double y, double r);
    virtual void setVelocity(double vx, double vy, double vr);
    virtual void setAngle(const std::vector<double>& q);
    virtual void setAngularVelocity(const std::vector<double>& dq);
    virtual void setTorque(const std::vector<double>& tau);
    virtual void getPosition(double& x, double& y, double& r);
    virtual void getVelocity(double& vx, double& vy, double& vr);
    virtual void getAngle(const std::vector<double>& q);
    virtual void getAngularVelocity(const std::vector<double>& dq);
    virtual void getTorque(const std::vector<double>& tau);

  private:

  };

}

#endif /* __AHL_YOUBOT_SERVER_GAZEBO_YOUBOT_BASE_HPP */
