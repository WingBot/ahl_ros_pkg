#include "ahl_robot_controller/robot/tf_publisher.hpp"

using namespace ahl_robot;

TfPublisher::TfPublisher()
{

}

tf2_ros::TransformBroadcaster& TfPublisher::transformBroadcaster()
{
  static tf2_ros::TransformBroadcaster br;
  return br;
}

void TfPublisher::publishJointFrames(const RobotPtr& robot)
{
  Joints::const_iterator it;
  for(it = robot->getJoints().begin(); it != robot->getJoints().end(); ++it)
  {
    std::string frame_id = it->first;
    JointPtr joint = it->second;


  }
}

void TfPublisher::publishLinkFrames(const RobotPtr& robot)
{

}
