#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <ahl_gazebo_interface/gazebo_interface.hpp>
#include <ahl_gazebo_interface/exception.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include <ahl_robot_controller/exception.hpp>
#include <ahl_robot_controller/robot_controller.hpp>
#include <ahl_robot_controller/tasks.hpp>

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
bool initialized = false;
bool joint_updated = false;

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
      tf_pub->publish(robot);
    }
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
    }

    if(updated)
    {
      static long cnt = 0;

      if(initialized == false)
      {
        Eigen::VectorXd qd = Eigen::VectorXd::Constant(robot->getDOF("mnp"), M_PI / 4.0);
        double sin_val = 1.0;//std::abs(sin(2.0 * M_PI * 0.1 * cnt * 0.001));
        qd = sin_val * qd;
        joint_control->setGoal(qd);

        static int reached = 0;
        if(robot->reached("mnp", qd, 0.03))
        {
          ++reached;

          if(reached > 2000)
          {
            initialized = true;
            controller->clearTask();
            controller->addTask(damping, 0);
            controller->addTask(gravity_compensation, 100);
            controller->addTask(position_control, 10);
            controller->addTask(orientation_control, 5);
            controller->addTask(joint_limit, 100);
            Eigen::Vector3d xd;
            xd << 0.35, 0.2, 0.8;
            position_control->setGoal(xd);
            orientation_control->setGoal(Eigen::Matrix3d::Identity());
          }
        }
        else
        {
          reached = 0;
        }
      }
      else
      {
        Eigen::Vector3d xd;
        xd << 0.35, 0.2, 0.8;
        position_control->setGoal(xd);
        orientation_control->setGoal(Eigen::Matrix3d::Identity());
      }

      ++cnt;

      Eigen::VectorXd tau(robot->getDOF("mnp"));
      controller->computeGeneralizedForce(tau);

      gazebo_interface->applyJointEfforts(tau);
    }

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

  ManipulatorPtr mnp = robot->getManipulator("mnp");

  gravity_compensation = TaskPtr(new GravityCompensation(mnp));
  damping = TaskPtr(new Damping(mnp));
  joint_control = TaskPtr(new JointControl(mnp));
  joint_limit = TaskPtr(new JointLimit(mnp, 0.087));
  position_control = TaskPtr(new PositionControl(mnp, "gripper", 0.001));
  orientation_control = TaskPtr(new OrientationControl(mnp, "gripper", 0.001));

  controller->addTask(gravity_compensation, 0);
  controller->addTask(damping, 0);
  controller->addTask(joint_control, 0);
  controller->addTask(joint_limit, 100);

  ros::MultiThreadedSpinner spinner;
  spinner.spin();

  return 0;
}
