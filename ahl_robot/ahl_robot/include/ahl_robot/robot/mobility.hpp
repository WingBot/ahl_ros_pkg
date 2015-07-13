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
