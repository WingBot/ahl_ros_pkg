#ifndef __AHL_ROBOT_MOBILITY_HPP
#define __AHL_ROBOT_MOBILITY_HPP

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Mobility
  {
  public:
    void print()
    {
      std::cout << "Mobility : " << std::endl
                << "q : " << std::endl << q << std::endl
                << "dq : " << std::endl << dq << std::endl
                << "type : " << type << std::endl
                << "command : " << command << std::endl
                << "cutoff_frequency  : " << cutoff_frequency << std::endl
                << "tread_width : " << tread_width << std::endl
                << "wheel_base : " << wheel_base << std::endl
                << "wheel_radius : " << wheel_radius << std::endl;
      for(unsigned int i = 0; i < joint_name.size(); ++i)
      {
        std::cout << "joint " << i << " : " << joint_name[i] << std::endl;
      }
    }

    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    std::vector<std::string> joint_name;
    std::string type;
    std::string command;
    double cutoff_frequency;
    double tread_width;
    double wheel_base;
    double wheel_radius;
  };

  typedef boost::shared_ptr<Mobility> MobilityPtr;
}

#endif /* __AHL_ROBOT_MOBILITY_HPP */
