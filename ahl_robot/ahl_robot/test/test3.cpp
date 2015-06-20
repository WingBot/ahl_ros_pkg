#include <ros/ros.h>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/parser.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"

using namespace ahl_robot;

Eigen::MatrixXd M;
Eigen::MatrixXd J;
Eigen::MatrixXd T0, T1, T2;

void calc()
{
  std::cout << "M : " << std::endl << M << std::endl
            << "J : " << std::endl << J << std::endl << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
    std::string name = "lwr";
    RobotPtr robot = RobotPtr(new Robot(name));

    ParserPtr parser = ParserPtr(new Parser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/lwr.yaml";
    parser->load(path, robot);

    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = TfPublisherPtr(new TfPublisher());

    const std::string mnp_name = "mnp";
    unsigned long cnt = 0;
    ros::Rate r(10.0);

    while(ros::ok())
    {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(robot->getDOF(mnp_name));
      ManipulatorPtr mnp = robot->getManipulator(mnp_name);

      double goal = sin(2.0 * M_PI * 0.2 * cnt * 0.1);
      ++cnt;
      for(unsigned int i = 0; i < q.rows(); ++i)
      {
        q.coeffRef(i) = M_PI / 4.0 * goal;
      }
      q = Eigen::VectorXd::Constant(q.rows(), 0.0 * M_PI / 4.0);

      robot->update(mnp_name, q);
      M = robot->getMassMatrix(mnp_name);
      J = robot->getBasicJacobian(mnp_name, "gripper");

      calc();
      tf_publisher->publish(robot, false);
      r.sleep();
    }
  }
  catch(ahl_robot::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
