#ifndef __AHL_ROBOT_CONTROLLER_PARAM_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_ctrl
{

  class Param
  {
  public:
    Param();
    double getKpJoint()
    {
      return kp_joint_;
    }
    double getKvJoint()
    {
      return kv_joint_;
    }
    double getKpTask()
    {
      return kp_task_;
    }
    double getKvTask()
    {
      return kv_task_;
    }
    const Eigen::VectorXd& getG() const
    {
      return g_;
    }

  private:
    double kp_joint_;
    double kv_joint_;
    double kp_task_;
    double kv_task_;
    Eigen::VectorXd g_;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_HPP */
