#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/task/joint_control.hpp>

using namespace ahl_robot;
using namespace ahl_ctrl;

boost::mutex mutex;
RobotPtr robot;
RobotControllerPtr controller;
bool updated = false;
TfPublisherPtr tf_pub;
TaskPtr task;
ahl_gazebo_if::GazeboInterfacePtr gazebo_interface;

void updateModel(const ros::TimerEvent&)
{
  try
  {
    boost::mutex::scoped_lock lock(mutex);
    if(gazebo_interface->subscribed())
    {
      Eigen::VectorXd q = gazebo_interface->getJointStates();
      robot->update("mnp", q);
      updated = true;
      tf_pub->publish(robot);
    }
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
    if(updated)
    {
      Eigen::VectorXd qd = Eigen::VectorXd::Constant(robot->getDOF("mnp"), M_PI / 2.0);
      static long cnt = 0;
      double sin_val = std::abs(sin(2.0 * M_PI * 0.1 * cnt * 0.001));
      ++cnt;
      qd = sin_val * qd;

      task->setGoal(qd);
      Eigen::VectorXd tau(robot->getDOF("mnp"));
      controller->computeGeneralizedForce(tau);

      gazebo_interface->applyJointEfforts(tau);
    }

  }
  catch(ahl_gazebo_if::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_server");
  ros::NodeHandle nh;

  ros::Timer timer_update_model = nh.createTimer(ros::Duration(0.01), updateModel);
  ros::Timer timer_control = nh.createTimer(ros::Duration(0.001), control);

  robot = RobotPtr(new Robot("lwr"));
  ParserPtr parser = ParserPtr(new Parser());

  std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/lwr.yaml";
  parser->load(path, robot);

  controller = RobotControllerPtr(new RobotController());
  controller->init(robot, "mnp");

  using namespace ahl_gazebo_if;
  gazebo_interface = GazeboInterfacePtr(new GazeboInterface());
  gazebo_interface->setDuration(0.010);
  gazebo_interface->addJoint("lwr::joint1");
  gazebo_interface->addJoint("lwr::joint2");
  gazebo_interface->addJoint("lwr::joint3");
  gazebo_interface->addJoint("lwr::joint4");
  gazebo_interface->addJoint("lwr::joint5");
  gazebo_interface->addJoint("lwr::joint6");
  gazebo_interface->addJoint("lwr::joint7");
  gazebo_interface->connect();

  tf_pub = TfPublisherPtr(new TfPublisher());

  task = TaskPtr(new JointControl(robot->getManipulator("mnp"), 0));
  controller->addTask(task);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
