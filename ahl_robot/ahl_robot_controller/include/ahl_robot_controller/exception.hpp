#ifndef __AHL_ROBOT_CONTROLLER_EXCEPTION_HPP
#define __AHL_ROBOT_CONTROLLER_EXCEPTION_HPP

#include <sstream>

namespace ahl_ctrl
{

  class Exception
  {
  public:
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {}

    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "ahl_ctrl::Exception occurred." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;
      return ss.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_EXCEPTION_HPP */
