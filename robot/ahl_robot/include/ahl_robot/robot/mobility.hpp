/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#ifndef __AHL_ROBOT_MOBILITY_HPP
#define __AHL_ROBOT_MOBILITY_HPP

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ahl_digital_filter/differentiator.hpp>

namespace ahl_robot
{

  class Mobility
  {
  public:
    Mobility();

    void print()
    {
      std::cout << "Mobility : " << std::endl
                << "p : " << p.transpose() << std::endl
                << "r : " << std::endl << r.x() << ", " << r.y() << ", " << r.z() << ", " << r.w() << std::endl
                << "v : " << v.transpose() << std::endl
                << "w : " << w.transpose() << std::endl
                << "q : " << q.transpose() << std::endl
                << "dq : " << dq.transpose() << std::endl
                << "type : " << type << std::endl
                << "command : " << command << std::endl
                << "cutoff_frequency_base : " << cutoff_frequency_base << std::endl
                << "cutoff_frequency_wheel : " << cutoff_frequency_wheel << std::endl
                << "tread_width : " << tread_width << std::endl
                << "wheel_base : " << wheel_base << std::endl
                << "wheel_radius : " << wheel_radius << std::endl;

      for(unsigned int i = 0; i < joint_name.size(); ++i)
      {
        std::cout << "joint " << i << " : " << joint_name[i] << std::endl;
      }
    }

    void init();
    void updateBase(const Eigen::Vector3d& p_msr, const Eigen::Quaternion<double>& r_msr);
    void updateBase(const Eigen::Vector3d& p_msr, const Eigen::Quaternion<double>& r_msr,
                    const Eigen::Vector3d& v_msr, const Eigen::Vector3d& w_msr);
    void updateWheel(const Eigen::VectorXd& q_msr);

    Eigen::VectorXd p;
    Eigen::Quaternion<double> r;
    Eigen::VectorXd v;
    Eigen::VectorXd w;

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    double update_rate;
    std::vector<std::string> joint_name;
    std::string type;
    std::string command;
    double cutoff_frequency_base;
    double cutoff_frequency_wheel;
    double tread_width;
    double wheel_base;
    double wheel_radius;

  private:
    ahl_filter::DifferentiatorPtr differentiator_pos_;
    ahl_filter::DifferentiatorPtr differentiator_wheel_;
    bool updated_pos_;
    bool updated_wheel_;
    ros::Time last_ori_update_time_;
  };

  typedef boost::shared_ptr<Mobility> MobilityPtr;
}

#endif /* __AHL_ROBOT_MOBILITY_HPP */
