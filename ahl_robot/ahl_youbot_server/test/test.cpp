#include <ros/ros.h>
#include <gazebo_msgs/ApplyJointEfforts.h>
#include <gazebo_msgs/JointStates.h>

void timerCB(const ros::TimerEvent&)
{
  gazebo_msgs::ApplyJointEfforts msg;
  msg.joint_name.push_back("arm_joint_1");
  msg.joint_name.push_back("arm_joint_2");
  msg.joint_name.push_back("arm_joint_3");
  msg.joint_name.push_back("arm_joint_4");
  msg.joint_name.push_back("arm_joint_5");

  static unsigned long cnt = 0;
  double f = 100.0;//5.0 * sin(2.0 * M_PI * 0.2 * cnt * 0.01);

  msg.effort.push_back(f);
  msg.effort.push_back(f);
  msg.effort.push_back(f);
  msg.effort.push_back(f);
  msg.effort.push_back(f);

  msg.start_time = ros::Time(0);
  msg.duration = ros::Duration(1);
  ROS_INFO_STREAM(f);
  ++cnt;
}

void callback(const gazebo_msgs::JointStates::ConstPtr& msg)
{

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<gazebo_msgs::ApplyJointEfforts>("/gazebo/apply_joint_efforts", 10);

  ros::Rate r(1000.0);

  while(ros::ok())
  {
    gazebo_msgs::ApplyJointEfforts msg;
    msg.joint_name.push_back("arm_joint_1");
    msg.joint_name.push_back("arm_joint_2");
    msg.joint_name.push_back("arm_joint_3");
    msg.joint_name.push_back("arm_joint_4");
    msg.joint_name.push_back("arm_joint_5");

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

    r.sleep();
  }

  ros::Subscriber sub = nh.subscribe("/gazebo/joint_states", 10, callback);
  ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCB);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
