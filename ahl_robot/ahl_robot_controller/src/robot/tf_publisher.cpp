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
  ros::Time current = ros::Time::now();

  for(Joints::const_iterator it = robot->getJoints().begin(); it != robot->getJoints().end(); ++it)
  {
    geometry_msgs::TransformStamped tf_stamped;

    std::string frame_id = it->first;
    JointPtr joint = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = joint->getName();

    if(it->second->getParentLink().get() != NULL)
    {
      LinkPtr parent_link;
      parent_link = joint->getParentLink();

      if(parent_link->getJoint().get() != NULL)
      {
        tf_stamped.header.frame_id = parent_link->getJoint()->getName();
      }
      else
        continue;
    }
    else
      continue;

    tf_stamped.transform.translation.x = joint->getT().coeff(0, 3);
    tf_stamped.transform.translation.y = joint->getT().coeff(1, 3);
    tf_stamped.transform.translation.z = joint->getT().coeff(2, 3);

    tf_stamped.transform.rotation.x = 0.0;
    tf_stamped.transform.rotation.y = 0.0;
    tf_stamped.transform.rotation.z = 0.0;
    tf_stamped.transform.rotation.w = 1.0;

/*
    std::cout << "parent : " << tf_stamped.header.frame_id << std::endl
              << "child  : " << tf_stamped.child_frame_id << std::endl;
    std::cout << tf_stamped.transform.translation.x << ", "
              << tf_stamped.transform.translation.y << ", "
              << tf_stamped.transform.translation.z << std::endl;
    std::cout << joint->org << std::endl;
*/

    transformBroadcaster().sendTransform(tf_stamped);
  }
}

void TfPublisher::publishLinkFrames(const RobotPtr& robot)
{
  ros::Time current = ros::Time::now();

  for(Links::const_iterator it = robot->getLinks().begin(); it != robot->getLinks().end(); ++it)
  {
    geometry_msgs::TransformStamped tf_stamped;

    std::string frame_id = it->first;
    LinkPtr link = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = link->getName();

    if(it->second->getJoint().get() != NULL)
    {
      JointPtr joint;
      joint = link->getJoint();

      if(joint->getParentLink().get() != NULL)
      {
        tf_stamped.header.frame_id = joint->getParentLink()->getName();
      }
      else
        continue;
    }
    else
      continue;

    tf_stamped.transform.translation.x = link->getT().coeff(0, 3);
    tf_stamped.transform.translation.y = link->getT().coeff(1, 3);
    tf_stamped.transform.translation.z = link->getT().coeff(2, 3);

    tf_stamped.transform.rotation.x = 0.0;
    tf_stamped.transform.rotation.y = 0.0;
    tf_stamped.transform.rotation.z = 0.0;
    tf_stamped.transform.rotation.w = 1.0;

/*
    std::cout << "parent : " << tf_stamped.header.frame_id << std::endl
              << "child  : " << tf_stamped.child_frame_id << std::endl;
    std::cout << tf_stamped.transform.translation.x << ", "
              << tf_stamped.transform.translation.y << ", "
              << tf_stamped.transform.translation.z << std::endl;
    std::cout << joint->org << std::endl;
*/

    transformBroadcaster().sendTransform(tf_stamped);
  }
}
