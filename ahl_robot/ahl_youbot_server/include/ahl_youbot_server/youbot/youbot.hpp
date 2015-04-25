#ifndef __AHL_YOUBOT_SERVER_YOUBOT_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_HPP

namespace ahl_youbot
{

  class YouBot
  {
  public:
    virtual ~YouBot() {}

    // Base
    virtual void doJointCommutation() = 0;
    virtual void getBasePosition(double& x, double& y, double& r) = 0;
    virtual void getBaseVelocity(double& vx, double& vy, double& vr) = 0;
    virtual void getJointData(std::vector<double>
    virtual void setBasePosition(double x, double y, double r) = 0;
    virtual void setBaseVelocity(double vx, double vy, double vr) = 0;


    virtual void getJointAngles(std::vector<double>& angles) = 0;
virtual 


  private:

  };

};

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_HPP */
