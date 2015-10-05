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

////////////////////////////////////////////////////////
/// \file gazebo_interface.hpp
/// \brief Declare ahl_gazebo_if::GazeboInterface class
/// \author Daichi Yoshikawa
////////////////////////////////////////////////////////

#ifndef __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP
#define __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP

#include <map>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <gazebo_msgs/AddJoint.h>
#include <gazebo_msgs/StartTimer.h>
#include <gazebo_msgs/LinkStates.h>
#include <ahl_utils/shared_memory.hpp>

namespace ahl_gazebo_if
{

  /// static const value which defines the topic name for link states
  static const std::string TOPIC_SUB_LINK_STATES  = "/gazebo/set_link_states";

  /// This class enables to communicate with gazebo simulator through ahl_utils::shared memory.
  class GazeboInterface
  {
  public:
    /// Constructor
    GazeboInterface();
    /// Destructor
    ~GazeboInterface();

    /// Register joint name to get joint angle or apply torque
    /// @param name Name of joint you'd like to use (mostly like pr2::shoulder_pan)
    /// @param effort_time Defines how long you'd like to apply effort to the joint when you call applyJointEfforts.
    void addJoint(const std::string& name, double effort_time = 0.010);

    /// This is unnecessary. 
    void setDuration(double duration);

    /// Initialize and connect the communication with gazebo simulator
    void connect();

    /// Check gazebo simulator has already written some value in shared memory.
    /// @return true : already written, false : not written yet
    bool subscribed();

    /// Apply torque to registered joint
    /// @param tau Torque vector. It should be aligned in order you added joint
    void applyJointEfforts(const Eigen::VectorXd& tau);

    /// Register link to translate or rotate without considering dynamics
    /// @param robot Name of robot you use (mostly like pr2)
    /// @param link Name of link you'd like to register (mostly like forearm)
    void addLink(const std::string& robot, const std::string& link);

    /// Translate link
    /// @param p xyz position vector w.r.t parent link frame
    void translateLink(const std::vector<Eigen::Vector3d>& p);

    /// Rotate link
    /// @param q Quartenion w.r.t parent link frame
    void rotateLink(const std::vector<Eigen::Quaternion<double> >& q);

    /// Get joint angles
    /// @return Joint state vector representing joint angles or displacements
    const Eigen::VectorXd& getJointStates();

  private:
    /// Mutex
    boost::mutex mutex_;

    /// Key : Joint name
    /// Value : Torque value 
    std::map<std::string, double> joint_map_;

    /// Key : Joint name
    /// Value : Joint index representing the registration order
    std::map<std::string, int> joint_to_idx_;

    /// Joint name list
    std::vector<std::string> joint_list_;

    /// Number of registered joint
    unsigned int joint_num_;

    /// Joint angle/displacement vector
    Eigen::VectorXd q_;

    /// Unnecessary variable
    ros::Duration duration_;

    /// True : Gazebo simulator has written some value to shared memory
    /// False : Gazebo simulator hasn't written any values to shared memory yet
    bool subscribed_joint_states_;

    /// Publisher to publish link position and orientation
    ros::Publisher pub_link_states_;

    /// Link positions and orientations
    gazebo_msgs::LinkStates link_states_;

    /// ROS service client to call service server provided by gazebo_ros to start writing joint angles/displacements
    ros::ServiceClient client_start_timer_;

    /// ROS service client to call service server provided by gazebo_ros to register joint to use
    ros::ServiceClient client_add_joint_;

    /// Key : Joint name
    /// Value : Torque to apply
    std::map<std::string, ahl_utils::SharedMemory<double>::Ptr > joint_effort_;

    /// Key : Joint name
    /// Value : Joint angle/displacement
    std::map<std::string, ahl_utils::SharedMemory<double>::Ptr > joint_state_;
  };

  typedef boost::shared_ptr<GazeboInterface> GazeboInterfacePtr;
}

#endif /* __AHL_GAZEBO_INTERFACE_GAZEBO_INTERFACE_HPP */
