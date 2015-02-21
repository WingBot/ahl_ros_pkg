/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
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

#include <sstream>
#include <ros/ros.h>
#include "train_with_cg/hand_image_collector.hpp"
#include "train_with_cg/exceptions.hpp"

using namespace train;

HandImageCollector::HandImageCollector()
{
  ros::NodeHandle local_nh("~");

  local_nh.param<double>("hand/scale", scale_, 0.01);
  local_nh.param<std::string>("hand/file_name", x_hand_file_name_, "");

  bool use_quaternion = false;

  local_nh.param<bool>("hand/use_quaternion", use_quaternion, false);

  std::vector<double> euler_max(3);
  std::vector<double> euler_min(3);
  double euler_step = 0.0; // [deg]

  local_nh.param<double>("hand/euler/x_max", euler_max[0], 90.0);
  local_nh.param<double>("hand/euler/y_max", euler_max[1], 90.0);
  local_nh.param<double>("hand/euler/z_max", euler_max[2], 90.0);
  local_nh.param<double>("hand/euler/x_min", euler_min[0], 0.0);
  local_nh.param<double>("hand/euler/y_min", euler_min[1], 0.0);
  local_nh.param<double>("hand/euler/z_min", euler_min[2], 0.0);
  local_nh.param<double>("hand/euler/step", euler_step, 30.0);

  std::vector<double> finger_max(5);
  std::vector<double> finger_min(5);
  double finger_step = 0.0;

  local_nh.param<double>("hand/finger/1st_max", finger_max[0], 90.0);
  local_nh.param<double>("hand/finger/2nd_max", finger_max[1], 90.0);
  local_nh.param<double>("hand/finger/3rd_max", finger_max[2], 90.0);
  local_nh.param<double>("hand/finger/4th_max", finger_max[3], 90.0);
  local_nh.param<double>("hand/finger/5th_max", finger_max[4], 90.0);
  local_nh.param<double>("hand/finger/1st_min", finger_min[0], 0.0);
  local_nh.param<double>("hand/finger/2nd_min", finger_min[1], 0.0);
  local_nh.param<double>("hand/finger/3rd_min", finger_min[2], 0.0);
  local_nh.param<double>("hand/finger/4th_min", finger_min[3], 0.0);
  local_nh.param<double>("hand/finger/5th_min", finger_min[4], 0.0);
  local_nh.param<double>("hand/finger/step", finger_step, 30.0);

  for(unsigned int i = 0; i < euler_max.size(); ++i)
  {
    euler_max[i] = euler_max[i] / 180.0 * M_PI;
    euler_min[i] = euler_min[i] / 180.0 * M_PI;
  }
  euler_step = euler_step / 180.0 * M_PI;

  for(unsigned int i = 0; i < finger_max.size(); ++i)
  {
    finger_max[i] = finger_max[i] / 180.0 * M_PI;
    finger_min[i] = finger_min[i] / 180.0 * M_PI;
  }
  finger_step = finger_step / 180.0 * M_PI;

  hand_pose_ = HandPosePtr(
    new HandPose(use_quaternion, euler_max, euler_min, euler_step, finger_max, finger_min, finger_step));

  depth_image_saver_ = DepthImageSaverPtr(new DepthImageSaver());
}

void HandImageCollector::collect()
{
  bool is_last = !hand_pose_->update();

  OrientationPtr orientation = hand_pose_->getOrientation();
  FingersPtr fingers         = hand_pose_->getFingers();

  if(orientation->isQuaternion())
  {
    double deg = 2.0 * std::acos(orientation->getOrientation().coeff(3, 0)) / M_PI * 180.0;

    glRotated(deg,
              orientation->getOrientation().coeff(0, 0),
              orientation->getOrientation().coeff(1, 0),
              orientation->getOrientation().coeff(2, 0));
  }
  else
  {
    double euler_z = orientation->getOrientation().coeff(2, 0) / M_PI * 180.0;
    double euler_y = orientation->getOrientation().coeff(1, 0) / M_PI * 180.0;
    double euler_x = orientation->getOrientation().coeff(0, 0) / M_PI * 180.0;

    glRotated(euler_z, 0.0, 0.0, 1.0);
    glRotated(euler_x, 1.0, 0.0, 0.0);
    glRotated(euler_y, 0.0, 1.0, 0.0);
  }

  this->getRightHand()->rotate1stFinger(fingers->getAngles().coeffRef(0, 0) / M_PI * 180.0);
  this->getRightHand()->rotate2ndFinger(fingers->getAngles().coeffRef(1, 0) / M_PI * 180.0);
  this->getRightHand()->rotate3rdFinger(fingers->getAngles().coeffRef(2, 0) / M_PI * 180.0);
  this->getRightHand()->rotate4thFinger(fingers->getAngles().coeffRef(3, 0) / M_PI * 180.0);
  this->getRightHand()->rotate5thFinger(fingers->getAngles().coeffRef(4, 0) / M_PI * 180.0);

  std::cout << (180.0 / M_PI) * orientation->getOrientation() << std::endl << std::endl;

  glScaled(scale_, scale_, scale_);
  this->getRightHand()->displayWithoutShade();

  static unsigned int cnt = 0;
  std::stringstream ss;
  ss << "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/data/depth/image" << cnt << ".bmp";
  depth_image_saver_->save(ss.str());
  ++cnt;

  if(is_last)
  {
    ros::shutdown();
    exit(0);
  }
}

bool HandImageCollector::finished()
{
  return false;
}

gl_wrapper::RightHandPtr& HandImageCollector::getRightHand()
{
  static gl_wrapper::RightHandPtr right_hand = gl_wrapper::RightHandPtr(new gl_wrapper::RightHand(x_hand_file_name_));
  return right_hand;
}
