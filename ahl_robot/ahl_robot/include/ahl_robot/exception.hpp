#ifndef __AHL_ROBOT_EXCEPTION_HPP
#define __AHL_ROBOT_EXCEPTION_HPP

#include <sstream>

namespace ahl_robot
{

  class Exception
  {
  public:
    Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg) {}

    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "ahl_robot::Exception occuered." << std::endl
                << "  src : " << src_ << std::endl
                << "  msg : " << msg_;

      return ss.str().c_str(); 
    }

    const std::string& getSrc() const
    {
      return src_;
    }

    const std::string& getMsg() const
    {
      return msg_;
    }

  private:
    std::string src_;
    std::string msg_;
  };

}

#endif /* __AHL_ROBOT_EXCEPTION_HPP */
