#include "ahl_robot/exception.hpp"
#include "ahl_robot/definition.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"

using namespace ahl_robot;

TfPublisher::TfPublisher()
{
}

void TfPublisher::publish(const RobotPtr& robot, bool publish_com)
{
  ros::Time current = ros::Time::now();

  std::vector<std::string>::const_iterator it;
  for(it = robot->getManipulatorName().begin(); it != robot->getManipulatorName().end(); ++it)
  {
    ManipulatorPtr mnp = robot->getManipulator(*it);
    this->publish(mnp, current, publish_com);
  }
}

void TfPublisher::publish(const ManipulatorPtr& mnp, const ros::Time& current, bool publish_com)
{
  if(mnp->T.size() != mnp->link.size())
  {
    std::stringstream msg;
    msg << "mnp->T.size() != mnp->link.size()" << std::endl
        << "  mnp->T.size()    = " << mnp->T.size() << std::endl
        << "  mnp->link.size() = " << mnp->link.size();
    throw ahl_robot::Exception("ahl_robot::TfPublisher::publish", msg.str());
  }

  for(unsigned int i = 0; i < mnp->T.size(); ++i)
  {
    geometry_msgs::TransformStamped tf_stamped;
    LinkPtr link = mnp->link[i];

    tf_stamped.header.frame_id = link->parent;
    tf_stamped.child_frame_id  = link->name;
    tf_stamped.header.stamp    = current;

    Eigen::Matrix3d R = mnp->T[i].block(0, 0, 3, 3);
    Eigen::Quaterniond q(R);

    tf_stamped.transform.translation.x = mnp->T[i].coeff(0, 3);
    tf_stamped.transform.translation.y = mnp->T[i].coeff(1, 3);
    tf_stamped.transform.translation.z = mnp->T[i].coeff(2, 3);

    tf_stamped.transform.rotation.x = q.x();
    tf_stamped.transform.rotation.y = q.y();
    tf_stamped.transform.rotation.z = q.z();
    tf_stamped.transform.rotation.w = q.w();
    transformBroadcaster().sendTransform(tf_stamped);

    if(!publish_com)
      continue;

    geometry_msgs::TransformStamped com_stamped;
    com_stamped.header.frame_id = link->name;
    com_stamped.child_frame_id  = link->name + "_com";
    com_stamped.header.stamp    = current;

    q = Eigen::Matrix3d::Identity();

    com_stamped.transform.translation.x = link->C.coeff(0);
    com_stamped.transform.translation.y = link->C.coeff(1);
    com_stamped.transform.translation.z = link->C.coeff(2);

    com_stamped.transform.rotation.x = q.x();
    com_stamped.transform.rotation.y = q.y();
    com_stamped.transform.rotation.z = q.z();
    com_stamped.transform.rotation.w = q.w();
    transformBroadcaster().sendTransform(com_stamped);
  }
}
tf2_ros::TransformBroadcaster& TfPublisher::transformBroadcaster()
{
  static tf2_ros::TransformBroadcaster br;
  return br;
}
