#ifndef __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP
#define __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP

#include "ahl_youbot_server/interface/interface.hpp"

namespace ahl_youbot
{

  class YoubotInterface : public Interface
  {
  public:
    YoubotInterface() {}

    bool getJointStates(Eigen::VectorXd& q) {}
    bool getJointStates(Eigen::VectorXd& q, Eigen::VectorXd& dq) {}
    bool applyJointEfforts(const Eigen::VectorXd& tau) {}
  };

}

#endif /* __AHL_YOUBOT_SERVER_YOUBOT_INTERFACE_HPP */
