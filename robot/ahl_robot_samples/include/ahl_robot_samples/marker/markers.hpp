#ifndef __AHL_ROBOT_SAMPLES_MARKERS_HPP
#define __AHL_ROBOT_SAMPLES_MARKERS_HPP

#include <map>
#include "ahl_robot_samples/marker/marker.hpp"

namespace ahl_sample
{

  class Markers
  {
  public:
    void add(const MarkerPtr& marker);
    void setColor(const std::string& name, int r, int g, int b, double a);
    void setColor(int r, int g, int b, double a);
    void setPosition(const std::string& name, double x, double y, double z);
    void setScale(const std::string& name, double scale);
    void setScale(double scale);
    void publish();
    void remove(const std::string& name);
    void remove();

  private:
    std::map<std::string, MarkerPtr> marker_;
  };

  typedef boost::shared_ptr<Markers> MarkersPtr;
}

#endif /* __AHL_ROBOT_SAMPLES_MARKERS_HPP */
