ahl_ros_pkg
===========
ahl_ros_pkg, which is a ROS package developed by Daichi Yoshikawa At Home Lab.
  It mainly composes of the following 3 things.
* Kinematics, Dynamics and Control of Robot
* Hand tracking system "[Manipit](http://www.instructables.com/id/Manipit-Hand-motion-tracking-with-painted-gloves/)"
* Others (utility, wrapper of OpenCV and OpenGL, digital filter and so on)

How does it work ?
==================
The remarkable packages are _"ahl_robot_controller"_, which contains [task space control (operational space control)](http://cs.stanford.edu/groups/manips/publications/pdfs/Khatib_1993_JSME.pdf) and _"neural_network"_.
  The following videos show how it works.

* Task space control of PR2.                   
[![](http://img.youtube.com/vi/7pHPHKFTwZs/0.jpg)](https://www.youtube.com/watch?v=7pHPHKFTwZs)

* 7DOF Hand pose tracking using neural network.      
[![](http://img.youtube.com/vi/nZZZX_Wu5kE/0.jpg)](https://www.youtube.com/watch?v=nZZZX_Wu5kE)

Contents
========
### Packages for robotics (robot, utils, filter, motion_planner)
----------------------------------------------------------------
* ahl_robot : Define robot with mobility and multiple arms, and compute Jacobian and Mass matrix 
* ahl_robot_controller : Control robot based on torque in joint space or operational space (task space)
* ahl_gazebo_interface : Enable to get joint angles from gazebo and apply torques to gazebo robot model through shared memory
* ahl_pr2_description : URDF description of PR2
* ahl_youbot_description : URDF description of Kuka YouBot
* ahl_red_arm : URDF description of original red arm
* ahl_robot_samples : Sample programs of task space control using PR2 and YouBot
* ahl_digital_filter : Digital filters like a pseudo differentiator

### Packages for manipit (machine_learning, manipit, utils)
-----------------------------------------------------------
* neural_network : Artificial neural network based on cpu computing
* gl_wrapper : Contains wrapper of OpenGL and provides skinned mesh parser
* cv_wrapper : Contains wrapper of OpenCV
* train_with_cg : Regression using neural network
* manipit : Hand tracking system using result of train_with_cg (Under development)

### Pre-existed ros packages
----------------------------
* gazebo_ros_pkgs : It's modified to enable to use shared memory for torque control
* openni2_camera : The same as the original
* openni2_launch : The same as the original
* cob_common, pr2_msgs, youbot_applilcations, youbot_driver : The same as the original and required to use physical YouBot

Other video clips
=================
* Task space control of mobile manipulator(Kuka YouBot).   
[![](http://img.youtube.com/vi/RHdLje50RXQ/0.jpg)](https://www.youtube.com/watch?v=RHdLje50RXQ)

* Task space control of 7 DOF manipulator.   
[![](http://img.youtube.com/vi/v_i-LgaJ5WM/0.jpg)](https://www.youtube.com/watch?v=v_i-LgaJ5WM)

* Task space control of 11 DOF manipulator.   
[![](http://img.youtube.com/vi/oKqCsFAzx4k/0.jpg)](https://www.youtube.com/watch?v=oKqCsFAzx4k)
