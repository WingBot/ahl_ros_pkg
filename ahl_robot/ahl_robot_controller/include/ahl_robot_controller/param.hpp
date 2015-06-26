#ifndef __AHL_ROBOT_CONTROLLER_PARAM_HPP
#define __AHL_ROBOT_CONTROLLER_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_controller/ParamConfig.h"

namespace ahl_ctrl
{

  class Param
  {
  public:
    Param();

    double getKp()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kp_;
    }

    double getKv()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kv_;
    }

    double getKvDamp()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kv_damp_;
    }

    double getKpLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kp_limit_;
    }

    double getKvLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return kv_limit_;
    }

    const Eigen::Vector3d& getG()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return g_;
    }

  private:
    void update(ahl_robot_controller::ParamConfig& config, uint32_t level);

    boost::mutex mutex_;

    dynamic_reconfigure::Server<ahl_robot_controller::ParamConfig> server_;
    dynamic_reconfigure::Server<ahl_robot_controller::ParamConfig>::CallbackType f_;

    double kp_;
    double kv_;
    double kv_damp_;
    double kp_limit_;
    double kv_limit_;
    Eigen::Vector3d g_;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARAM_HPP */
