#ifndef __AHL_ROBOT_CONTROLLER_LINK_HPP
#define __AHL_ROBOT_CONTROLLER_LINK_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace ahl_robot
{

  class Link;
  typedef boost::shared_ptr<Link> LinkPtr;

  class Link
  {
  public:
    Link();
    void print();

    std::string name;

    Eigen::Matrix3d I;
    Eigen::Matrix4d org;
    Eigen::Matrix4d com;
    double M;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_LINK_HPP */
