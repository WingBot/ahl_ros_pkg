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
      double coeff = 0.5 * sin(2.0 * M_PI * 0.1 * cnt * 0.1);
      ++cnt;

      q = coeff * q;
      q.coeffRef(0) = coeff;
      q.coeffRef(1) = coeff;
      q.coeffRef(2) = coeff;

      robot->update(mnp_name, q);
      Eigen::VectorXd dq = robot->getJointVelocity(mnp_name);
      Eigen::MatrixXd J0 = robot->getBasicJacobian(mnp_name);
      std::cout << J0 * dq << std::endl << std::endl;
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
