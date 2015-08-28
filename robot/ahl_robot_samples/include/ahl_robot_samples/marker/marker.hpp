#ifndef __AHL_ROBOT_SAMPLES_MARKER_HPP
#define __AHL_ROBOT_SAMPLES_MARKER_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace ahl_sample
{

  class Marker
  {
  public:
    explicit Marker(const std::string& name, const std::string& frame, uint32_t shape = visualization_msgs::Marker::SPHERE);

    void setColor(int r, int g, int b, double a);
    void setPosition(double x, double y, double z);
    void setScale(double scale);
    void publish();
    void remove();
    const std::string& getName() const
    {
      return marker_.ns;
    }

  private:
    ros::Publisher pub_;
    visualization_msgs::Marker marker_;

  };

  typedef boost::shared_ptr<Marker> MarkerPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_MARKER_HPP */
