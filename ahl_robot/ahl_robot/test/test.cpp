#include <ros/ros.h>
#include "ahl_robot/exception.hpp"
#include "ahl_robot/robot/parser.hpp"
#include "ahl_robot/tf/tf_publisher.hpp"

using namespace ahl_robot;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parser_test");
  ros::NodeHandle nh;

  try
  {
    std::string name = "youbot";
    RobotPtr robot = RobotPtr(new Robot(name));

    ParserPtr parser = ParserPtr(new Parser());
    std::string path = "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ahl_robot/ahl_robot/yaml/youbot.yaml";
    parser->load(path, robot);

    ros::MultiThreadedSpinner spinner;

    TfPublisherPtr tf_publisher = TfPublisherPtr(new TfPublisher());

    const std::string mnp_name = "mnp";
    unsigned long cnt = 0;
    ros::Rate r(10.0);

    while(ros::ok())
    {
      Eigen::VectorXd q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), 1.0);
      double coeff = 1.0 * sin(2.0 * M_PI * 0.1 * cnt * 0.1);
      ++cnt;

      q = coeff * q;
      //q = Eigen::VectorXd::Constant(robot->getDOF(mnp_name), M_PI / 2.0);
      q.coeffRef(0) = 0.0;
      q.coeffRef(1) = 0.0;
      q.coeffRef(2) = 0.0;

      robot->update(mnp_name, q);
      robot->computeBasicJacobian(mnp_name);
      robot->computeMassMatrix(mnp_name);

      Eigen::VectorXd dq = robot->getJointVelocity(mnp_name);
      Eigen::MatrixXd J0 = robot->getBasicJacobian(mnp_name);

      std::cout << dq << std::endl << std::endl;
      std::cout << cos(2.0 * M_PI * 0.1 * cnt * 0.1) << std::endl;

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
