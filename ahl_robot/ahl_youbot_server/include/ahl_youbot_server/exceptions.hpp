#ifndef __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP
#define __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP

#include <sstream>

namespace ahl_youbot
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
      ss << "ahl_youbot::Exception was occured." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

  class FatalException
  {
  public:
    explicit FatalException(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg)
    {}

    const char* what() const throw()
    {
      std::stringstream ss;
      ss << "ahl_youbot::FatalException was occured." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
  };
}

#endif /* __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP */
