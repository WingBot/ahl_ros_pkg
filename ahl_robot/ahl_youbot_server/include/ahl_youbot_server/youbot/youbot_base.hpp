#ifndef __AHL_YOUBOT_SERVER_YOUBOT_BASE_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_BASE_HPP

#include <vector>
#include <boost/shared_ptr.hpp>

namespace ahl_youbot
{

  class YouBotBase
  {
  public:
    virtual ~YouBotBase() {}

    virtual void doJointCommutation() = 0;
    virtual void setPosition(double x, double y, double r) = 0;
    virtual void setVelocity(double vx, double vy, double vr) = 0;
    virtual void setAngle(const std::vector<double>& q) = 0;
    virtual void setAngularVelocity(const std::vector<double>& dq) = 0;
    virtual void setTorque(const std::vector<double>& tau) = 0;
    virtual void getPosition(double& x, double& y, double& r) = 0;
    virtual void getVelocity(double& vx, double& vy, double& vr) = 0;
    virtual void getAngle(const std::vector<double>& q) = 0;
    virtual void getAngularVelocity(const std::vector<double>& dq) = 0;
    virtual void getTorque(const std::vector<double>& tau) = 0;
  };

  typedef boost::shared_ptr<YouBotBase> AHLYouBotBasePtr;
}

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_BASE_HPP */
