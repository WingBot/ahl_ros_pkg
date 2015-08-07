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

#include <fstream>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_robot_controller/exception.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/tasks.hpp>
#include <ahl_youbot_server/param.hpp>

using namespace ahl_robot;
using namespace ahl_ctrl;

boost::mutex mutex;
RobotPtr robot;
RobotControllerPtr controller;
bool updated = false;
TfPublisherPtr tf_pub;
TaskPtr gravity_compensation;
TaskPtr damping;
TaskPtr joint_control;
TaskPtr joint_limit;
TaskPtr position_control;
TaskPtr orientation_control;
ahl_gazebo_if::GazeboInterfacePtr gazebo_interface;
ahl_gazebo_if::GazeboInterfacePtr gazebo_interface_wheel;
ahl_gazebo_if::GazeboInterfacePtr gazebo_interface_base;
bool initialized = false;
bool joint_updated = false;
Eigen::VectorXd tau_wheel;
bool tau_computed = false;
Eigen::VectorXd pose_base;

void updateWheel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex);
    if(gazebo_interface_wheel->subscribed() && gazebo_interface_base->subscribed())
    {
      if(tau_wheel.rows() != 3)
        return;

      if(!updated)
        return;

      Eigen::VectorXd q = gazebo_interface_wheel->getJointStates();
      robot->updateWheel(q);

      Eigen::Vector3d base_pos;
      base_pos << pose_base[0], pose_base[1], 0.0;
      Eigen::Quaternion<double> base_ori;
      base_ori.x() = 0.0;
      base_ori.y() = 0.0;
      base_ori.z() = sin(0.5 * pose_base[2]);
      base_ori.w() = cos(0.5 * pose_base[2]);

      robot->updateBase(base_pos, base_ori);

      Eigen::VectorXd v_base;
      controller->computeBaseVelocityFromTorque(tau_wheel, v_base, 3);

      Eigen::VectorXd v_wheel;
      controller->computeWheelVelocityFromBaseVelocity(v_base, v_wheel);

      Eigen::VectorXd q_wheel_d = robot->getMobility()->update_rate * v_wheel;

      std::vector<Eigen::Quaternion<double> > quat_d;
      for(unsigned int i = 0; i < q_wheel_d.rows(); ++i)
      {
        double rad_wheel = q_wheel_d.coeff(i);

        Eigen::Quaternion<double> quat;
        quat.x() = 0.0;
        quat.y() = sin(rad_wheel * 0.5);
        quat.z() = 0.0;
        quat.w() = cos(rad_wheel * 0.5);

        quat_d.push_back(quat);
      }

      gazebo_interface_wheel->rotateLink(quat_d);
    }
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void updateModel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex);
    if(joint_updated)
    {
      robot->computeBasicJacobian("mnp");
      robot->computeMassMatrix("mnp");
      controller->updateModel();
      updated = true;
    }
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void control(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex);

    if(gazebo_interface->subscribed())
    {
      Eigen::VectorXd q = gazebo_interface->getJointStates();
      robot->update("mnp", q);
      joint_updated = true;

      pose_base = q.block(0, 0, 3, 1);
    }

    if(updated)
    {
      static long cnt = 0;

      if(initialized == false)
      {
        Eigen::VectorXd qd = Eigen::VectorXd::Constant(robot->getDOF("mnp"), M_PI / 4.0);
        double sin_val = 1.0;//std::abs(sin(2.0 * M_PI * 0.1 * cnt * 0.001));
        qd = sin_val * qd;
        qd.coeffRef(0) = 0.5;
        qd.coeffRef(1) = 0.0;
        qd.coeffRef(2) = 0.0;
        joint_control->setGoal(qd);

        static int reached = 0;
        if(robot->reached("mnp", qd, 0.05))
        {
          ++reached;

          if(reached > 500)
          {
            initialized = true;
            controller->clearTask();
            //controller->addTask(damping, 0);
            controller->addTask(joint_control, 0);
            controller->addTask(gravity_compensation, 10);
            controller->addTask(position_control, 10);
            controller->addTask(orientation_control, 5);
            //controller->addTask(joint_limit, 100);
            Eigen::Vector3d xd;
            xd << 0.4, 0.15, 0.1;

            position_control->setGoal(xd);

            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
            double rad = 0.0;
            R << cos(rad), 0, sin(rad),
              0, 1, 0,
              -sin(rad), 0, cos(rad);
            orientation_control->setGoal(R);
          }
        }
        else
        {
          reached = 0;
        }
      }
      else
      {
        Eigen::VectorXd qd(robot->getDOF("mnp"));
        qd.coeffRef(0) = robot->getManipulator("mnp")->q.coeff(0);
        qd.coeffRef(1) = robot->getManipulator("mnp")->q.coeff(1);
        qd.coeffRef(2) = robot->getManipulator("mnp")->q.coeff(2) + robot->getManipulator("mnp")->q.coeff(3);
        qd.coeffRef(3) = 0.0;
        qd.coeffRef(4) = M_PI / 6.0;
        qd.coeffRef(5) = M_PI / 6.0;
        qd.coeffRef(6) = M_PI / 6.0;
        qd.coeffRef(7) = 0.0;
        joint_control->setGoal(qd);

        Eigen::Vector3d xd;
        xd << 2.4, 0.15, 0.07;
        //xd.coeffRef(0) += 0.2 * sin(2.0 * M_PI * 0.1 * cnt * 0.001);
        xd.coeffRef(1) += 0.02 * sin(2.0 * M_PI * 0.2 * cnt * 0.001);
        xd.coeffRef(2) += 0.07 * sin(2.0 * M_PI * 0.2 * cnt * 0.001);

        position_control->setGoal(xd);
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        double rad = M_PI;
        R << cos(rad), 0, sin(rad),
          0, 1, 0,
          -sin(rad), 0, cos(rad);

        orientation_control->setGoal(R);
      }

      ++cnt;

      Eigen::VectorXd tau(robot->getDOF("mnp"));
      Eigen::Vector3d cmd_vel;
      controller->computeGeneralizedForce(tau);
      gazebo_interface->applyJointEfforts(tau);
      tau_wheel = tau.block(0, 0, 3, 1);
    }
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "youbot_test");
  ros::NodeHandle nh;

  ros::Timer timer_update_wheel = nh.createTimer(ros::Duration(0.01), updateWheel);
  ros::Timer timer_update_model = nh.createTimer(ros::Duration(0.01), updateModel);
  ros::Timer timer_control = nh.createTimer(ros::Duration(0.001), control);

  try
  {
    robot = RobotPtr(new Robot("youbot"));
    ParserPtr parser = ParserPtr(new Parser());

    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/youbot.yaml";
    parser->load(path, robot);

    controller = RobotControllerPtr(new RobotController());
    controller->init(robot, "mnp");

    using namespace ahl_gazebo_if;
    gazebo_interface = GazeboInterfacePtr(new GazeboInterface());
    gazebo_interface->setDuration(0.01);
    gazebo_interface->addJoint("youbot::base_x_joint");
    gazebo_interface->addJoint("youbot::base_y_joint");
    gazebo_interface->addJoint("youbot::base_yaw_joint");
    gazebo_interface->addJoint("youbot::joint1");
    gazebo_interface->addJoint("youbot::joint2");
    gazebo_interface->addJoint("youbot::joint3");
    gazebo_interface->addJoint("youbot::joint4");
    gazebo_interface->addJoint("youbot::joint5");
    gazebo_interface->connect();

    tau_wheel = Eigen::Vector3d::Zero();
    gazebo_interface_wheel = GazeboInterfacePtr(new GazeboInterface());
    gazebo_interface_wheel->setDuration(1.0);
    gazebo_interface_wheel->addJoint("youbot::wheel_joint_fl");
    gazebo_interface_wheel->addJoint("youbot::wheel_joint_fr");
    gazebo_interface_wheel->addJoint("youbot::wheel_joint_bl");
    gazebo_interface_wheel->addJoint("youbot::wheel_joint_br");
    gazebo_interface_wheel->addLink("youbot", "wheel_link_fl");
    gazebo_interface_wheel->addLink("youbot", "wheel_link_fr");
    gazebo_interface_wheel->addLink("youbot", "wheel_link_bl");
    gazebo_interface_wheel->addLink("youbot", "wheel_link_br");
    gazebo_interface_wheel->connect();

    gazebo_interface_base = GazeboInterfacePtr(new GazeboInterface());
    gazebo_interface_base->addJoint("youbot::base_x_joint");
    gazebo_interface_base->addJoint("youbot::base_y_joint");
    gazebo_interface_base->addJoint("youbot::base_yaw_joint");
    gazebo_interface_base->connect();

    tf_pub = TfPublisherPtr(new TfPublisher());

    ManipulatorPtr mnp = robot->getManipulator("mnp");

    gravity_compensation = TaskPtr(new GravityCompensation(robot));
    damping = TaskPtr(new Damping(mnp));
    joint_control = TaskPtr(new JointControl(mnp));
    joint_limit = TaskPtr(new JointLimit(mnp, 0.087));
    position_control = TaskPtr(new PositionControl(mnp, "gripper", 0.001));
    orientation_control = TaskPtr(new OrientationControl(mnp, "gripper", 0.001));

    controller->addTask(gravity_compensation, 0);
    controller->addTask(joint_control, 0);
    //controller->addTask(joint_limit, 100);

    ros::MultiThreadedSpinner spinner;
    spinner.spin();
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }

  return 0;
}
