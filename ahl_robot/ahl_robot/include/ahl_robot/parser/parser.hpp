#ifndef __AHL_ROBOT_PARSER_HPP
#define __AHL_ROBOT_PARSER_HPP

#include <string>
#include <boost/shared_ptr.hpp>
#include "ahl_robot/robot/robot.hpp"

namespace ahl_robot
{

  class Parser
  {
  public:
    Parser() {}
    virtual ~Parser() {}
    virtual void load(const std::string& path, RobotPtr& robot) {}

  protected:
  };

  typedef boost::shared_ptr<Parser> ParserPtr;
}

#endif /* __AHL_ROBOT_PARSER_HPP */
