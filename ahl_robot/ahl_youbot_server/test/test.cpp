/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

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
