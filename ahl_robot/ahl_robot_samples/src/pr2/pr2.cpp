#include "ahl_robot_samples/pr2/pr2.hpp"

using namespace ahl_sample;

PR2::PR2()
{

}

void PR2::init()
{
  initRobot("pr2", "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot_samples/yaml/pr2.yaml");

  controller_ = RobotControllerPtr(new RobotController());
  controller_->init(robot_);

  ManipulatorPtr mnp_l = robot_->getManipulator("left_mnp");
  //ManipulatorPtr mnp_r = robot_->getManipulator("right_mnp");

  gravity_compensation_l_   = TaskPtr(new GravityCompensation(mnp_l));
  //gravity_compensation_r_   = TaskPtr(new GravityCompensation(mnp_r));
  joint_control_l_          = TaskPtr(new JointControl(mnp_l));
  //joint_control_r_          = TaskPtr(new JointControl(mnp_r));
  joint_limit_l_            = TaskPtr(new JointLimit(mnp_l, 0.087));
  //joint_limit_r_            = TaskPtr(new JointLimit(mnp_r, 0.087));
  position_control_l_       = TaskPtr(new PositionControl(mnp_l, "gripper_l_link", 0.001));
  //position_control_r_       = TaskPtr(new PositionControl(mnp_r, "gripper_r_link", 0.001));
  //position_control_base_    = TaskPtr(new PositionControl(mnp_l, "base_yaw", 0.001));
  orientation_control_l_    = TaskPtr(new OrientationControl(mnp_l, "gripper_l_link", 0.001));
  //orientation_control_r_    = TaskPtr(new OrientationControl(mnp_r, "gripper_r_link", 0.001));
  //orientation_control_base_ = TaskPtr(new OrientationControl(mnp_l, "base_yaw", 0.001));

  Eigen::VectorXd q_l = Eigen::VectorXd::Zero(mnp_l->dof);
  //Eigen::VectorXd q_r = Eigen::VectorXd::Zero(mnp_r->dof);
  //Eigen::MatrixXd R_base = Eigen::Matrix3d::Identity();
  //Eigen::VectorXd x_base = Eigen::Vector3d::Zero();
  Eigen::MatrixXd R_l = Eigen::Matrix3d::Identity();
  //Eigen::MatrixXd R_r = Eigen::Matrix3d::Identity();
  Eigen::VectorXd x_l = Eigen::Vector3d::Zero();
  //Eigen::VectorXd x_r = Eigen::Vector3d::Zero();

  x_l << 0.5, 0.3, 0.6;
  //x_r << 0.5, -0.3, 0.6;

  joint_control_l_->setGoal(q_l);
  //joint_control_r_->setGoal(q_r);
  //orientation_control_base_->setGoal(R_base);
  //position_control_base_->setGoal(x_base);
  orientation_control_l_->setGoal(R_l);
  //orientation_control_r_->setGoal(R_r);
  position_control_l_->setGoal(x_l);
  //position_control_r_->setGoal(x_r);

  //controller_->addTask(joint_control_l_, 0);
  //controller_->addTask(joint_control_r_, 1);
  //controller_->addTask(orientation_control_base_, 10);
  //controller_->addTask(position_control_base_, 11);
  //controller_->addTask(orientation_control_l_, 20);
  //controller_->addTask(orientation_control_r_, 21);
  //controller_->addTask(position_control_l_, 30);
  controller_->addTask(gravity_compensation_l_, 30);
  //controller_->addTask(position_control_r_, 31);
  //controller_->addTask(gravity_compensation_r_, 30);
  //controller_->addTask(joint_limit_l_, 40);
  //controller_->addTask(joint_limit_r_, 41);

  gazebo_interface_ = GazeboInterfacePtr(new GazeboInterface());
  gazebo_interface_->setDuration(0.01);
  gazebo_interface_->addJoint("pr2::base_x_joint");
  gazebo_interface_->addJoint("pr2::base_y_joint");
  gazebo_interface_->addJoint("pr2::base_yaw_joint");
  gazebo_interface_->addJoint("pr2::torso_joint");
  gazebo_interface_->addJoint("pr2::shoulder_pan_l_joint");
  gazebo_interface_->addJoint("pr2::shoulder_lift_l_joint");
  gazebo_interface_->addJoint("pr2::upper_arm_l_joint");
  gazebo_interface_->addJoint("pr2::elbow_l_joint");
  gazebo_interface_->addJoint("pr2::forearm_l_joint");
  gazebo_interface_->addJoint("pr2::wrist_flex_l_joint");
  gazebo_interface_->addJoint("pr2::wrist_roll_l_joint");
  gazebo_interface_->addJoint("pr2::shoulder_pan_r_joint");
  gazebo_interface_->addJoint("pr2::shoulder_lift_r_joint");
  gazebo_interface_->addJoint("pr2::upper_arm_r_joint");
  gazebo_interface_->addJoint("pr2::elbow_r_joint");
  gazebo_interface_->addJoint("pr2::forearm_r_joint");
  gazebo_interface_->addJoint("pr2::wrist_flex_r_joint");
  gazebo_interface_->addJoint("pr2::wrist_roll_r_joint");

  gazebo_interface_->connect();

  tf_pub_ = TfPublisherPtr(new TfPublisher());
}

void PR2::run()
{
  ros::NodeHandle nh;

  timer_update_model_ = nh.createTimer(ros::Duration(0.01), &PR2::updateModel, this);
  timer_control_ = nh.createTimer(ros::Duration(0.001), &PR2::control, this);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();
}

void PR2::updateModel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);
    if(!joint_updated_) return;

    robot_->computeBasicJacobian();
    robot_->computeMassMatrix();
    controller_->updateModel();

    Eigen::MatrixXd J = robot_->getBasicJacobian("right_mnp");
    Eigen::MatrixXd M = robot_->getMassMatrix("right_mnp");

    model_updated_ = true;
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

void PR2::control(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex_);

    if(gazebo_interface_->subscribed())
    {
      Eigen::VectorXd q = gazebo_interface_->getJointStates();
      robot_->update(q);
      joint_updated_ = true;
    }

    if(!model_updated_) return;

    Eigen::VectorXd tau;
    controller_->computeGeneralizedForce(tau);
    //tau.block(0, 0, 4, 1) *= 0.50;
    gazebo_interface_->applyJointEfforts(tau);
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(ahl_ctrl::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}
