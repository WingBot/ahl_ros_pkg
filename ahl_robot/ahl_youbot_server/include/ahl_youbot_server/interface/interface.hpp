#ifndef __AHL_YOUBOT_SERVER_INTERFACE_HPP
#define __AHL_YOUBOT_SERVER_INTERFACE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_youbot
{

  class Interface
  {
  public:
    enum Type
    {
      YOUBOT_INTERFACE,
      GAZEBO_INTERFACE,
      INTERFACE_NUM,
    };

    virtual ~Interface() {}
    virtual void getJointStates(Eigen::VectorXd& q) = 0;
    virtual void getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq) = 0;
    virtual void applyJointEfforts(const Eigen::VectorXd& tau) = 0;
  };

  typedef boost::shared_ptr<Interface> InterfacePtr;
}

#endif /* __AHL_YOUBOT_SERVER_INTERFACE_HPP */
