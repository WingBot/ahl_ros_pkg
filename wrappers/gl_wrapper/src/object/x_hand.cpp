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

#include <gl_wrapper/object/x_hand.hpp>

using namespace gl_wrapper;

XHandCore::XHandCore()
{
  std::string config_file_name;
  nh_.param<std::string>("/xhand/config_file_name", config_file_name, "/home/daichi/work/catkin_ws/src/gl_wrapper/righthand.cfg");
  if(config_file_name == "")
  {
    ROS_ERROR("Please set \"/xhand/config_file_name\" in launch file");
    ros::shutdown();
    exit(0);
  }

  xhand.init(config_file_name);
}

XDeformableObject& XHand::getLeftHand()
{
  static XHandCore xhand_core;
  return xhand_core.getXHand();
}

void XHand::rotateAllFingers(double deg)
{
  rotate1stFinger(deg);
  rotate2ndFinger(deg);
  rotate3rdFinger(deg);
  rotate4thFinger(deg);
  rotate5thFinger(deg);
}

void XHand::rotate1stFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  double rad1 = 0.25*rad;
  double rad2 = 0.55*rad;
  double rad3 = rad;

  Eigen::Matrix3d Rx1;
  Rx1 << 1.0, 0.0,        0.0,
         0.0, cos(rad1), -sin(rad1),
         0.0, sin(rad1),  cos(rad1);
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,        0.0,
         0.0, cos(rad2), -sin(rad2),
         0.0, sin(rad2),  cos(rad2);
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);

  getLeftHand().rotate("f11", Rx1);
  getLeftHand().rotate("f12", Rx2);
  getLeftHand().rotate("f13", Rx3);
}

void XHand::rotate2ndFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = 0.04*rad;
  double rad21 = 0.05*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f21", Rz1);
  getLeftHand().rotate("f22", R2);
  getLeftHand().rotate("f23", Rx3);
  getLeftHand().rotate("f24", Rx4);
}

void XHand::rotate3rdFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  Eigen::Matrix3d Rx;
  Rx << 1.0, 0.0,       0.0,
        0.0, cos(rad), -sin(rad),
        0.0, sin(rad),  cos(rad);

  getLeftHand().rotate("f32", Rx);
  getLeftHand().rotate("f33", Rx);
  getLeftHand().rotate("f34", Rx);
}

void XHand::rotate4thFinger(double deg)
{
  double rad = deg/180.0*M_PI;
  double rad21 = -0.2*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f42", R2);
  getLeftHand().rotate("f43", Rx3);
  getLeftHand().rotate("f44", Rx4);
}

void XHand::rotate5thFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = -0.06*rad;
  double rad21 = -0.4*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,         0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,         0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f51", Rz1);
  getLeftHand().rotate("f52", R2);
  getLeftHand().rotate("f53", Rx3);
  getLeftHand().rotate("f54", Rx4);
}
/*
void XHand::rotateOrientation(Quaternion& quat)
{
  double deg = 0.0;
  quat.getRotationAngle(deg);
  Eigen::Vector3d axis;
  axis << quat.getX(), quat.getY(), quat.getZ();
  axis = axis.normalized();

  glRotated(deg, axis[0], axis[1], axis[2]);
}
*/
DeformableHandCore::DeformableHandCore(std::string file_name)
{
  std::string config_file_name;
  if(file_name == "")
  {
    nh_.getParam("/xhand/config_file_name", config_file_name);
    if(config_file_name == "")
    {
      ROS_ERROR("Please set \"/xhand/config_file_name\" in launch file");
      ros::shutdown();
      exit(0);
    }
  }
  else
  {
    config_file_name = file_name;
  }

  hand_.init(config_file_name);
}

void DeformableHand::rotateAllFingers(double deg)
{
  rotate1stFinger(deg);
  rotate2ndFinger(deg);
  rotate3rdFinger(deg);
  rotate4thFinger(deg);
  rotate5thFinger(deg);
}
/*
void DeformableHand::rotateOrientation(Quaternion& quat)
{
  double deg = 0.0;
  quat.getRotationAngle(deg);
  Eigen::Vector3d axis;
  axis << quat.getX(), quat.getY(), quat.getZ();
  axis = axis.normalized();

  glRotated(deg, axis[0], axis[1], axis[2]);
}
*/
void RightHand::rotate1stFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  double rad1 = 0.25*rad;
  double rad2 = 0.55*rad;
  double rad3 = rad;

  Eigen::Matrix3d Rx1;
  Rx1 << 1.0, 0.0,        0.0,
         0.0, cos(rad1), -sin(rad1),
         0.0, sin(rad1),  cos(rad1);
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,        0.0,
         0.0, cos(rad2), -sin(rad2),
         0.0, sin(rad2),  cos(rad2);
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);

  getRightHand().rotate("f11", Rx1);
  getRightHand().rotate("f12", Rx2);
  getRightHand().rotate("f13", Rx3);
}

void RightHand::rotate2ndFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = 0.04*rad;
  double rad21 = 0.05*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getRightHand().rotate("f21", Rz1);
  getRightHand().rotate("f22", R2);
  getRightHand().rotate("f23", Rx3);
  getRightHand().rotate("f24", Rx4);
}

void RightHand::rotate3rdFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  Eigen::Matrix3d Rx;
  Rx << 1.0, 0.0,       0.0,
        0.0, cos(rad), -sin(rad),
        0.0, sin(rad),  cos(rad);

  getRightHand().rotate("f32", Rx);
  getRightHand().rotate("f33", Rx);
  getRightHand().rotate("f34", Rx);
}

void RightHand::rotate4thFinger(double deg)
{
  double rad = deg/180.0*M_PI;
  double rad21 = -0.2*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getRightHand().rotate("f42", R2);
  getRightHand().rotate("f43", Rx3);
  getRightHand().rotate("f44", Rx4);
}

void RightHand::rotate5thFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = -0.06*rad;
  double rad21 = -0.4*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,         0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,         0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getRightHand().rotate("f51", Rz1);
  getRightHand().rotate("f52", R2);
  getRightHand().rotate("f53", Rx3);
  getRightHand().rotate("f54", Rx4);
}

void RightHand::display()
{
  this->getRightHand().display();
}

void RightHand::displayWithoutShade()
{
  this->getRightHand().displayWithoutShade();
}

XDeformableObject& RightHand::getRightHand()
{
  static DeformableHandCore core(file_name_);
  return core.getHand();
}

void LeftHand::rotate1stFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  double rad1 = -0.25*rad;
  double rad2 = -0.55*rad;
  double rad3 = -rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad2), -sin(rad2), 0.0,
         sin(rad2),  cos(rad2), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz3;
  Rz3 << cos(rad3), -sin(rad3), 0.0,
         sin(rad3),  cos(rad3), 0.0,
         0.0,        0.0,       1.0;

  getLeftHand().rotate("f11", Rz1);
  getLeftHand().rotate("f12", Rz2);
  getLeftHand().rotate("f13", Rz3);

/*
  Matrix<3, 3, double> Rx1(1.0, 0.0,        0.0,
                           0.0, cos(rad1), -sin(rad1),
                           0.0, sin(rad1),  cos(rad1));
  Matrix<3, 3, double> Rx2(1.0, 0.0,        0.0,
                           0.0, cos(rad2), -sin(rad2),
                           0.0, sin(rad2),  cos(rad2));
  Matrix<3, 3, double> Rx3(1.0, 0.0,        0.0,
                           0.0, cos(rad3), -sin(rad3),
                           0.0, sin(rad3),  cos(rad3));

  getLeftHand().rotate("f11", Rx1);
  getLeftHand().rotate("f12", Rx2);
  getLeftHand().rotate("f13", Rx3);
*/
}

void LeftHand::rotate2ndFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = -0.04*rad;
  double rad21 = -0.05*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f21", Rz1);
  getLeftHand().rotate("f22", R2);
  getLeftHand().rotate("f23", Rx3);
  getLeftHand().rotate("f24", Rx4);
}

void LeftHand::rotate3rdFinger(double deg)
{
  double rad  = deg/180.0*M_PI;
  Eigen::Matrix3d Rx;
  Rx << 1.0, 0.0,       0.0,
        0.0, cos(rad), -sin(rad),
        0.0, sin(rad),  cos(rad);

  getLeftHand().rotate("f32", Rx);
  getLeftHand().rotate("f33", Rx);
  getLeftHand().rotate("f34", Rx);
}

void LeftHand::rotate4thFinger(double deg)
{
  double rad = deg/180.0*M_PI;
  double rad21 = 0.2*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,        0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,        0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f42", R2);
  getLeftHand().rotate("f43", Rx3);
  getLeftHand().rotate("f44", Rx4);
}

void LeftHand::rotate5thFinger(double deg)
{
  double rad   = deg/180.0*M_PI;
  double rad1  = 0.06*rad;
  double rad21 = 0.4*rad;
  double rad22 = rad;
  double rad3  = rad;
  double rad4  = rad;

  Eigen::Matrix3d Rz1;
  Rz1 << cos(rad1), -sin(rad1), 0.0,
         sin(rad1),  cos(rad1), 0.0,
         0.0,        0.0,       1.0;
  Eigen::Matrix3d Rz2;
  Rz2 << cos(rad21), -sin(rad21), 0.0,
         sin(rad21),  cos(rad21), 0.0,
         0.0,         0.0,        1.0;
  Eigen::Matrix3d Rx2;
  Rx2 << 1.0, 0.0,         0.0,
         0.0, cos(rad22), -sin(rad22),
         0.0, sin(rad22),  cos(rad22);
  Eigen::Matrix3d R2 = Rz2 * Rx2;
  Eigen::Matrix3d Rx3;
  Rx3 << 1.0, 0.0,         0.0,
         0.0, cos(rad3), -sin(rad3),
         0.0, sin(rad3),  cos(rad3);
  Eigen::Matrix3d Rx4;
  Rx4 << 1.0, 0.0,         0.0,
         0.0, cos(rad4), -sin(rad4),
         0.0, sin(rad4),  cos(rad4);

  getLeftHand().rotate("f51", Rz1);
  getLeftHand().rotate("f52", R2);
  getLeftHand().rotate("f53", Rx3);
  getLeftHand().rotate("f54", Rx4);
}

void LeftHand::display()
{
  this->getLeftHand().display();
}

void LeftHand::displayWithoutShade()
{
  this->getLeftHand().displayWithoutShade();
}

XDeformableObject& LeftHand::getLeftHand()
{
  static DeformableHandCore core(file_name_);
  return core.getHand();
}
