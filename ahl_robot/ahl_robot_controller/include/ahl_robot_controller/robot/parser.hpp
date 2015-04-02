#ifndef __AHL_ROBOT_CONTROLLER_PARSER_HPP
#define __AHL_ROBOT_CONTROLLER_PARSER_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include "ahl_robot_controller/robot/robot.hpp"

namespace ahl_robot
{

  class Parser
  {
  public:
    virtual ~Parser() {}
    virtual void load(const std::string& file_name, const std::string& robot_name, RobotPtr& robot) {}
  };

  typedef boost::shared_ptr<Parser> ParserPtr;
}

#endif /* __AHL_ROBOT_CONTROLLER_PARSER_HPP */
