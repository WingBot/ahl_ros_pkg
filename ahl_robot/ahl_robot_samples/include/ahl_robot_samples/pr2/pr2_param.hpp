#ifndef __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP
#define __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ahl_robot_samples/Pr2ParamConfig.h"

namespace ahl_sample
{

  class Pr2Param
  {
  public:
    Pr2Param()
    {
      const unsigned int dof_l = 11;
      const unsigned int dof_r = 11;

      show_target = true;
      sin_x_l = false;
      sin_y_l = false;
      sin_z_l = false;
      sin_x_r = false;
      sin_y_r = false;
      sin_z_r = false;
      xl = Eigen::Vector3d::Zero();
      Rl = Eigen::Matrix3d::Identity();
      xr = Eigen::Vector3d::Zero();
      Rr = Eigen::Matrix3d::Identity();
      q_l = Eigen::VectorXd::Zero(dof_l);
      q_r = Eigen::VectorXd::Zero(dof_r);

      ros::NodeHandle local_nh("~/pr2/");
      f_ = boost::bind(&Pr2Param::update, this, _1, _2);
      server_ = Pr2ParamConfigServerPtr(new Pr2ParamConfigServer(local_nh));
      server_->setCallback(f_);
    }

    bool show_target;
    bool sin_x_l;
    bool sin_y_l;
    bool sin_z_l;
    bool sin_x_r;
    bool sin_y_r;
    bool sin_z_r;
    Eigen::Vector3d xl;
    Eigen::Matrix3d Rl;
    Eigen::Vector3d xr;
    Eigen::Matrix3d Rr;
    Eigen::VectorXd q_l;
    Eigen::VectorXd q_r;
  private:
    void update(ahl_robot_samples::Pr2ParamConfig& config, uint32_t level)
    {
      show_target = config.show_target;

      sin_x_l = config.sin_x_l;
      sin_y_l = config.sin_y_l;
      sin_z_l = config.sin_z_l;

      sin_x_r = config.sin_x_r;
      sin_y_r = config.sin_y_r;
      sin_z_r = config.sin_z_r;

      xl[0] = config.x_l;
      xl[1] = config.y_l;
      xl[2] = config.z_l;

      Rl = Eigen::AngleAxisd(config.roll_l,  Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(config.pitch_l, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(config.yaw_l,   Eigen::Vector3d::UnitZ());

      xr[0] = config.x_r;
      xr[1] = config.y_r;
      xr[2] = config.z_r;

      Rr = Eigen::AngleAxisd(config.roll_r,  Eigen::Vector3d::UnitX())
         * Eigen::AngleAxisd(config.pitch_r, Eigen::Vector3d::UnitY())
         * Eigen::AngleAxisd(config.yaw_r,   Eigen::Vector3d::UnitZ());

      q_l[0]  = config.q_base1;
      q_l[1]  = config.q_base2;
      q_l[2]  = config.q_base3;
      q_l[3]  = config.q_base4;
      q_l[4]  = config.q_l1;
      q_l[5]  = config.q_l2;
      q_l[6]  = config.q_l3;
      q_l[7]  = config.q_l4;
      q_l[8]  = config.q_l5;
      q_l[9]  = config.q_l6;
      q_l[10] = config.q_l7;

      q_r[0]  = config.q_base1;
      q_r[1]  = config.q_base2;
      q_r[2]  = config.q_base3;
      q_r[3]  = config.q_base4;
      q_r[4]  = config.q_r1;
      q_r[5]  = config.q_r2;
      q_r[6]  = config.q_r3;
      q_r[7]  = config.q_r4;
      q_r[8]  = config.q_r5;
      q_r[9]  = config.q_r6;
      q_r[10] = config.q_r7;
    }

    typedef dynamic_reconfigure::Server<ahl_robot_samples::Pr2ParamConfig> Pr2ParamConfigServer;
    typedef boost::shared_ptr<Pr2ParamConfigServer> Pr2ParamConfigServerPtr;

    Pr2ParamConfigServerPtr server_;
    dynamic_reconfigure::Server<ahl_robot_samples::Pr2ParamConfig>::CallbackType f_;
  };

  typedef boost::shared_ptr<Pr2Param> Pr2ParamPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_PR2_PARAM_HPP */
