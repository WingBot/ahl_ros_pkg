#ifndef __KALMAN_FILTER_EXCEPTION_HPP
#define __KALMAN_FILTER_EXCEPTION_HPP

#include <sstream>

namespace kf
{

  class Exception
  {
  public:
    Exception(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg) {}

    std::string what()
    {
      std::stringstream msg;

      msg << "kf::Exception was thrown." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return msg.str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

}

#endif /* __KALMAN_FILTER_EXCEPTION_HPP */
