#include "ahl_robot/exception.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"

using namespace ahl_robot;

TfPublisher::TfPublisher()
{

}

void TfPublisher::publish(const RobotPtr& robot)
{
  ros::Time current = ros::Time::now();

  this->publishManipulators(robot, current);
  this->publishJointFrames(robot, current);
  this->publishLinkFrames(robot, current);
}

void TfPublisher::publishManipulators(const RobotPtr& robot, const ros::Time& current)
{
  std::map<std::string, ManipulatorPtr>::iterator it;

  for(it = robot->mnp.begin(); it != robot->mnp.end(); ++it)
  {
    std::string mnp_name = it->first;
    ManipulatorPtr mnp = it->second;

    for(unsigned int i = 1; i < mnp->link.size(); ++i)
    {
      geometry_msgs::TransformStamped tf_stamped;
      tf_stamped.header.stamp = current;
      tf_stamped.child_frame_id = mnp->link[i]->name;

      tf_stamped.header.frame_id = mnp->link[i - 1]->name;

      tf_stamped.transform.translation.x = mnp->link[i]->T.coeff(0, 3);
      tf_stamped.transform.translation.y = mnp->link[i]->T.coeff(1, 3);
      tf_stamped.transform.translation.z = mnp->link[i]->T.coeff(2, 3);

      Eigen::Matrix3d R = mnp->link[i]->T.block(0, 0, 3, 3);
      Eigen::Quaterniond q(R);

      tf_stamped.transform.rotation.x = q.x();
      tf_stamped.transform.rotation.y = q.y();
      tf_stamped.transform.rotation.z = q.z();
      tf_stamped.transform.rotation.w = q.w();

      transformBroadcaster().sendTransform(tf_stamped);
    }
  }
}

void TfPublisher::publishJointFrames(const RobotPtr& robot, const ros::Time& current)
{
  for(TfJointMap::const_iterator it = robot->tf_joint.begin(); it != robot->tf_joint.end(); ++it)
  {
    geometry_msgs::TransformStamped tf_stamped;
    TfJointPtr joint = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = joint->name;

    TfLinkPtr parent_link = joint->parent_link;
    if(!parent_link)
      continue;

    tf_stamped.header.frame_id = robot->base_name;//parent_link->name;

    tf_stamped.transform.translation.x = joint->T.coeff(0, 3);
    tf_stamped.transform.translation.y = joint->T.coeff(1, 3);
    tf_stamped.transform.translation.z = joint->T.coeff(2, 3);

    Eigen::Matrix3d R = joint->T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    transformBroadcaster().sendTransform(tf_stamped);
  }
}

void TfPublisher::publishLinkFrames(const RobotPtr& robot, const ros::Time& current)
{
  for(TfLinkMap::const_iterator it = robot->tf_link.begin(); it != robot->tf_link.end(); ++it)
  {
    geometry_msgs::TransformStamped tf_stamped;
    TfLinkPtr link = it->second;

    tf_stamped.header.stamp = current;
    tf_stamped.child_frame_id = link->name;

    TfJointPtr joint = link->joint;
    if(!joint)
      continue;

    TfLinkPtr parent_link = joint->parent_link;
    if(!parent_link)
      continue;

    tf_stamped.header.frame_id = robot->base_name;//parent_link->name;

    Eigen::Matrix3d R = link->T.block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.translation.x = link->T.coeff(0, 3);
    tf_stamped.transform.translation.y = link->T.coeff(1, 3);
    tf_stamped.transform.translation.z = link->T.coeff(2, 3);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();

    geometry_msgs::TransformStamped com_stamped;
    com_stamped.header.frame_id = robot->base_name;//link->name;
    com_stamped.header.stamp = current;
    com_stamped.child_frame_id = link->name + "_com";

    R = link->com.block(0, 0, 3, 3);
    q = R;

    com_stamped.transform.translation.x = link->com.coeff(0, 3);
    com_stamped.transform.translation.y = link->com.coeff(1, 3);
    com_stamped.transform.translation.z = link->com.coeff(2, 3);

    com_stamped.transform.rotation.x = q.x();
    com_stamped.transform.rotation.y = q.y();
    com_stamped.transform.rotation.z = q.z();
    com_stamped.transform.rotation.w = q.w();

    transformBroadcaster().sendTransform(tf_stamped);
    transformBroadcaster().sendTransform(com_stamped);
  }
}

tf2_ros::TransformBroadcaster& TfPublisher::transformBroadcaster()
{
  static tf2_ros::TransformBroadcaster br;
  return br;
}
