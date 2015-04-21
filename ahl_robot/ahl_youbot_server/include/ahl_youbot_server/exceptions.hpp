#ifndef __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP
#define __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP

#include <sstream>

namespace ahl_youbot
{

  class Exception
  {
  public:
    Exception(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg)
    {}

    std::string what()
    {
      std::stringstream ss;
      ss << "ahl_youbot::Exception was occured." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

  class FatalException
  {
  public:
    FatalException(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg)
    {}

    std::string what()
    {
      std::stringstream ss;
      ss << "ahl_youbot::FatalException was occured." << std::endl
         << "  src : " << src_ << std::endl
         << "  msg : " << msg_;

      return ss.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };
}

#endif /* __AHL_YOUBOT_SERVER_EXCEPTIONS_HPP */
