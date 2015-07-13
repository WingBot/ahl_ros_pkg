#ifndef __AHL_YOUBOT_SERVER_PARAM_HPP
#define __AHL_YOUBOT_SERVER_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <ahl_robot_controller/param.hpp>
#include "ahl_youbot_server/ParamConfig.h"

namespace ahl_youbot
{

  class Param// : public ahl_ctrl::Param
  {
  public:
    Param();

    const Eigen::Vector3d& getG()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return g_;
    }

    const Eigen::MatrixXd& getKp()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_;
    }

    const Eigen::MatrixXd& getKv()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_;
    }

    const Eigen::MatrixXd& getKvDamp()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_damp_;
    }

    const Eigen::MatrixXd& getKpLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_limit_;
    }

    const Eigen::MatrixXd& getKvLimit()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_limit_;
    }

    const Eigen::MatrixXd& getKpWheel()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kp_wheel_;
    }

    const Eigen::MatrixXd& getKvWheel()
    {
      boost::mutex::scoped_lock lock(mutex_);
      return Kv_wheel_;
    }

  private:
    void update(ahl_youbot_server::ParamConfig& config, uint32_t level);

    boost::mutex mutex_;
    dynamic_reconfigure::Server<ahl_youbot_server::ParamConfig> server_;
    dynamic_reconfigure::Server<ahl_youbot_server::ParamConfig>::CallbackType f_;
    ahl_ctrl::ParamPtr param_;

    Eigen::Vector3d g_;
    Eigen::MatrixXd Kp_;
    Eigen::MatrixXd Kv_;
    Eigen::MatrixXd Kv_damp_;
    Eigen::MatrixXd Kp_limit_;
    Eigen::MatrixXd Kv_limit_;
    Eigen::MatrixXd Kp_wheel_;
    Eigen::MatrixXd Kv_wheel_;
  };

  typedef boost::shared_ptr<Param> ParamPtr;
}

#endif /* __AHL_YOUBOT_SERVER_PARAM_HPP */
