#ifndef __AHL_ROBOT_CONTROLLER_TF_PUBLISHER_HPP
#define __AHL_ROBOT_CONTROLLER_TF_PUBLISHER_HPP

#include <boost/shared_ptr.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "ahl_robot_controller/robot/robot.hpp"

namespace ahl_robot
{

  class TfPublisher
  {
  public:
    TfPublisher();
    void publishJointFrames(const RobotPtr& robot);
    void publishLinkFrames(const RobotPtr& robot); // Frame attached to COM of a link

  private:
    tf2_ros::TransformBroadcaster& transformBroadcaster();
  };

  typedef boost::shared_ptr<TfPublisher> TfPublisherPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_TF_PUBLISHER_HPP */
