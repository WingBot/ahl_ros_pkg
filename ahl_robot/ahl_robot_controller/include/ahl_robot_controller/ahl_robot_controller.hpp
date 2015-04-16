#ifndef __AHL_ROBOT_CONTROLLER_AHL_ROBOT_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_AHL_ROBOT_CONTROLLER_HPP

#include <boost/shared_ptr.hpp>
#include "ahl_robot_controller/robot/robot.hpp"
#include "ahl_robot_controller/robot/parser.hpp"
#include "ahl_robot_controller/robot/tf_publisher.hpp"

namespace ahl_robot
{

  class AHLRobotController
  {
  public:
    AHLRobotController();

  private:
    void publishTfCB(const ros::TimerEvent&);

    ParserPtr parser_;
    RobotPtr robot_;
    TfPublisherPtr tf_publisher_;

    bool publish_joint_frames_;
    bool publish_link_frames_;
    ros::Timer timer_publish_tf_;
  };

  typedef boost::shared_ptr<AHLRobotController> AHLRobotControllerPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_AHL_ROBOT_CONTROLLER_HPP */
