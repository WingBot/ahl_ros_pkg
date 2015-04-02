#ifndef __AHL_ROBOT_CONTROLLER_EXCEPTIONS_HPP
#define __AHL_ROBOT_CONTROLLER_EXCEPTIONS_HPP

#include <sstream>

namespace ahl_robot
{

  class Exception
  {
  public:
    Exception(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg), name_(std::string("ahl_robot::Exception")) {}
    virtual ~Exception() {}

    virtual std::string what()
    {
      std::stringstream msg;

      msg << name_ << " occured." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return msg.str();
    }

  protected:
    virtual void setName(const std::string& name)
    {
      name_ = name;
    }

  private:
    std::string src_;
    std::string msg_;
    std::string name_;
  };

  class FatalException : public Exception
  {
  public:
    FatalException(const std::string& src, const std::string& msg)
      : Exception(src, msg)
    {
      Exception::setName(std::string("ahl_robot::FatalException"));
    }
  };

}

#endif /* __AHL_ROBOT_CONTROLLER_EXCEPTIONS_HPP */
