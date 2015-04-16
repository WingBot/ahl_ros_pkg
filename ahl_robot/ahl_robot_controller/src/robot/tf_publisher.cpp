#include "ahl_robot_controller/exceptions.hpp"
#include "ahl_robot_controller/robot/tf_publisher.hpp"
#include "ahl_robot_controller/utils/math.hpp"

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

    JointPtr joint = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = joint->getName();

    LinkPtr parent_link = joint->getParentLink();
    if(!parent_link)
      continue;

    tf_stamped.header.frame_id = parent_link->getName();

    tf_stamped.transform.translation.x = joint->getT().coeff(0, 3);
    tf_stamped.transform.translation.y = joint->getT().coeff(1, 3);
    tf_stamped.transform.translation.z = joint->getT().coeff(2, 3);

    Eigen::Matrix3d R = joint->getT().block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    transformBroadcaster().sendTransform(tf_stamped);
  }
}

void TfPublisher::publishLinkFrames(const RobotPtr& robot)
{
  ros::Time current = ros::Time::now();

  for(Links::const_iterator it = robot->getLinks().begin(); it != robot->getLinks().end(); ++it)
  {
    geometry_msgs::TransformStamped tf_stamped;
    LinkPtr link = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = link->getName();

    JointPtr joint = link->getJoint();
    if(!joint)
      continue;

    LinkPtr parent_link = joint->getParentLink();
    if(!parent_link)
      continue;

    tf_stamped.header.frame_id = parent_link->getName();

    Eigen::Matrix3d R = link->getT().block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.translation.x = link->getT().coeff(0, 3);
    tf_stamped.transform.translation.y = link->getT().coeff(1, 3);
    tf_stamped.transform.translation.z = link->getT().coeff(2, 3);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    geometry_msgs::TransformStamped com_stamped;
    com_stamped.header.frame_id = link->getName();
    com_stamped.header.stamp = current;
    com_stamped.child_frame_id = link->getName() + "_com";

    R = link->getCOM().block(0, 0, 3, 3);
    q = R;

    com_stamped.transform.translation.x = link->getCOM().coeff(0, 3);
    com_stamped.transform.translation.y = link->getCOM().coeff(1, 3);
    com_stamped.transform.translation.z = link->getCOM().coeff(2, 3);

    com_stamped.transform.rotation.x = q.x();
    com_stamped.transform.rotation.y = q.y();
    com_stamped.transform.rotation.z = q.z();
    com_stamped.transform.rotation.w = q.w();
/*
    std::cout << "parent : " << tf_stamped.header.frame_id << std::endl
              << "child  : " << tf_stamped.child_frame_id << std::endl;
    std::cout << tf_stamped.transform.translation.x << ", "
              << tf_stamped.transform.translation.y << ", "
              << tf_stamped.transform.translation.z << std::endl;
    std::cout << joint->org << std::endl;
*/

    transformBroadcaster().sendTransform(tf_stamped);
    transformBroadcaster().sendTransform(com_stamped);
  }
}
