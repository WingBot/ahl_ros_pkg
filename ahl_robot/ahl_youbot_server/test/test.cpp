#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEfforts.h>
#include <gazebo_msgs/JointStates.h>

void callback(const gazebo_msgs::JointStates::ConstPtr& msg)
{
  ROS_INFO_STREAM(msg->name.size());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<gazebo_msgs::ApplyJointEfforts>("/gazebo/apply_joint_efforts", 10);

  ros::Rate r(1000.0);
  ros::Subscriber sub = nh.subscribe("/gazebo/joint_states", 10, callback);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  while(ros::ok())
  {
    gazebo_msgs::ApplyJointEfforts msg;
    msg.name.push_back("arm_joint_1");
    msg.name.push_back("arm_joint_2");
    msg.name.push_back("arm_joint_3");
    msg.name.push_back("arm_joint_4");
    msg.name.push_back("arm_joint_5");

    static unsigned long cnt = 0;
    double f = 5.0 * sin(2.0 * M_PI * 0.1 * cnt * 0.01);

    msg.effort.push_back(0);
    msg.effort.push_back(f);
    msg.effort.push_back(f);
    msg.effort.push_back(f);
    msg.effort.push_back(f);

    msg.start_time = ros::Time(0);
    msg.duration = ros::Duration(0.001);
    ROS_INFO_STREAM(f);
    ++cnt;

    pub.publish(msg);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
