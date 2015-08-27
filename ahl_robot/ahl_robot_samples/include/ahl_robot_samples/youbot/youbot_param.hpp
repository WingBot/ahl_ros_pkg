#ifndef __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP
#define __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_samples/YouBotParamConfig.h"

namespace ahl_sample
{

  class YouBotParam
  {
  public:
    YouBotParam()
    {
      const unsigned int dof = 8;

      show_target = true;
      sin_x = false;
      sin_y = false;
      sin_z = false;
      x_arm = Eigen::Vector3d::Zero();
      R_arm = Eigen::Matrix3d::Zero();
      x_base = Eigen::Vector3d::Zero();
      R_base = Eigen::Matrix3d::Zero();
      q = Eigen::VectorXd::Zero(dof);

      ros::NodeHandle local_nh("~/youbot/");
      f_ = boost::bind(&YouBotParam::update, this, _1, _2);
      server_ = YouBotParamConfigServerPtr(new YouBotParamConfigServer(local_nh));
      server_->setCallback(f_);
    }

    bool show_target;
    bool sin_x;
    bool sin_y;
    bool sin_z;
    Eigen::Vector3d x_arm;
    Eigen::Matrix3d R_arm;
    Eigen::Vector3d x_base;
    Eigen::Matrix3d R_base;
    Eigen::VectorXd q;
  private:
    void update(ahl_robot_samples::YouBotParamConfig& config, uint32_t level)
    {
      show_target = config.show_target;

      sin_x = config.sin_x;
      sin_y = config.sin_y;
      sin_z = config.sin_z;

      x_arm[0] = config.x_arm;
      x_arm[1] = config.y_arm;
      x_arm[2] = config.z_arm;

      R_arm = Eigen::AngleAxisd(config.roll_arm,  Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(config.pitch_arm, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(config.yaw_arm,   Eigen::Vector3d::UnitZ());

      x_base[0] = config.x_base;
      x_base[1] = config.y_base;
      x_base[2] = config.z_base;

      R_base = Eigen::AngleAxisd(config.roll_base,  Eigen::Vector3d::UnitX())
             * Eigen::AngleAxisd(config.pitch_base, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(config.yaw_base,   Eigen::Vector3d::UnitZ());

      q[0] = config.q_base1;
      q[1] = config.q_base2;
      q[2] = config.q_base3;
      q[3] = config.q1;
      q[4] = config.q2;
      q[5] = config.q3;
      q[6] = config.q4;
      q[7] = config.q5;
    }

    typedef dynamic_reconfigure::Server<ahl_robot_samples::YouBotParamConfig> YouBotParamConfigServer;
    typedef boost::shared_ptr<YouBotParamConfigServer> YouBotParamConfigServerPtr;

    YouBotParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_samples::YouBotParamConfig>::CallbackType f_;
  };

  typedef boost::shared_ptr<YouBotParam> YouBotParamPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_YOUBOT_PARAM_HPP */
