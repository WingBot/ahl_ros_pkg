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

#ifndef __GL_WRAPPER_OBJECT_XHAND_HPP
#define __GL_WRAPPER_OBJECT_XHAND_HPP

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <gl_wrapper/gl_wrapper.hpp>
#include <gl_wrapper/object/x_deformable_object.hpp>

#include <ros/ros.h>

namespace gl_wrapper
{
  class XHandCore
  {
  public:
    XHandCore();
    XDeformableObject& getXHand()
    {
      return xhand;
    }

  private:
    ros::NodeHandle nh_;
    XDeformableObject xhand;
  };

  class XHand
  {
  public:
    void rotateAllFingers(double deg);
    void rotate1stFinger(double deg);
    void rotate2ndFinger(double deg);
    void rotate3rdFinger(double deg);
    void rotate4thFinger(double deg);
    void rotate5thFinger(double deg);
    //void rotateOrientation(Quaternion& quat);
    void display()
    {
      getLeftHand().display();
    }
    void displayWithoutShade()
    {
      getLeftHand().displayWithoutShade();
    }

  private:
    XDeformableObject& getLeftHand();

    ros::NodeHandle nh_;
  };

  class DeformableHandCore
  {
  public:
    DeformableHandCore(std::string file_name_ = "");
    XDeformableObject& getHand()
    {
      return hand_;
    }

  private:
    ros::NodeHandle nh_;
    XDeformableObject hand_;
  };

  class DeformableHand
  {
  public:
    virtual ~DeformableHand() {}

    virtual void rotateAllFingers(double deg);
    virtual void rotate1stFinger(double deg) = 0;
    virtual void rotate2ndFinger(double deg) = 0;
    virtual void rotate3rdFinger(double deg) = 0;
    virtual void rotate4thFinger(double deg) = 0;
    virtual void rotate5thFinger(double deg) = 0;
    //virtual void rotateOrientation(Quaternion& quat);
    virtual void display() = 0;
    virtual void displayWithoutShade() = 0;

  private:
    virtual XDeformableObject& getRightHand() {}
    virtual XDeformableObject& getLeftHand() {}

  protected:
  };

  class RightHand : public DeformableHand
  {
  public:
    virtual void rotate1stFinger(double deg);
    virtual void rotate2ndFinger(double deg);
    virtual void rotate3rdFinger(double deg);
    virtual void rotate4thFinger(double deg);
    virtual void rotate5thFinger(double deg);
    RightHand(std::string file_name = "") : file_name_(file_name) {}
    void display();
    void displayWithoutShade();

  private:
    XDeformableObject& getRightHand();
    std::string file_name_;
  };

  class LeftHand : public DeformableHand
  {
  public:
    virtual void rotate1stFinger(double deg);
    virtual void rotate2ndFinger(double deg);
    virtual void rotate3rdFinger(double deg);
    virtual void rotate4thFinger(double deg);
    virtual void rotate5thFinger(double deg);
    LeftHand(std::string file_name = "") : file_name_(file_name) {}
    void display();
    void displayWithoutShade();

  private:
    XDeformableObject& getLeftHand();
    std::string file_name_;
  };

  typedef boost::shared_ptr<XHand> XHandPtr;
  typedef boost::shared_ptr<DeformableHand> DeformableHandPtr;
  typedef boost::shared_ptr<RightHand> RightHandPtr;
  typedef boost::shared_ptr<LeftHand> LeftHandPtr;
}

#endif
