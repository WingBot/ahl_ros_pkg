#ifndef __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_ctrl
{

  class ParamBase
  {
  public:
    virtual ~ParamBase() {}

    virtual const Eigen::MatrixXd& getKpJoint() = 0;
    virtual const Eigen::MatrixXd& getKvJoint() = 0;
    virtual const Eigen::MatrixXd& getKpTask() = 0;
    virtual const Eigen::MatrixXd& getKvTask() = 0;
    virtual const Eigen::MatrixXd& getKvDamp() = 0;
    virtual const Eigen::MatrixXd& getKpLimit() = 0;
    virtual const Eigen::MatrixXd& getKpWheel() = 0;
    virtual const Eigen::MatrixXd& getKvWheel() = 0;
    virtual const Eigen::Vector3d& getG() = 0;
  };

  typedef boost::shared_ptr<ParamBase> ParamBasePtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_BASE_HPP */
