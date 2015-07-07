#ifndef __AHL_ROBOT_CONTROLLER_MOBILITY_HPP
#define __AHL_ROBOT_CONTROLLER_MOBILITY_HPP

namespace ahl_ctrl
{

  class Mobility
  {
  public:
    virtual ~Mobility() {}

    virtual void computeWheelVelocity(const Eigen::Vector3d& v_base, Eigen::VectorXd& v_wheel) = 0;
    virtual void setFilter(double period, double cutoff_freq) {}
    virtual void computeWheelTorques(const Eigen::Vector3d& v_base, Eigen::VectorXd& v_wheel) = 0;
  private:
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_MOBILITY_HPP */
