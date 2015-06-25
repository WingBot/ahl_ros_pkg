#ifndef __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP
#define __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP

#include <map>
#include <list>
#include <boost/shared_ptr.hpp>
#include <ahl_robot/ahl_robot.hpp>
#include "ahl_robot_controller/task/task.hpp"
#include "ahl_robot_controller/task/multi_task.hpp"

namespace ahl_ctrl
{

  class RobotController
  {
  public:
    RobotController();
    void init(const ahl_robot::RobotPtr& robot, const std::string& mnp_name);
    void addTask(const TaskPtr& task, int priority);
    void clearTask();
    void updateModel();
    void computeGeneralizedForce(Eigen::VectorXd& tau);

  private:
    MultiTaskPtr multi_task_;
    std::vector<TaskPtr> task_;
    ahl_robot::ManipulatorPtr mnp_;
    Eigen::VectorXd tau_;
  };

  typedef boost::shared_ptr<RobotController> RobotControllerPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_ROBOT_CONTROLLER_HPP */
