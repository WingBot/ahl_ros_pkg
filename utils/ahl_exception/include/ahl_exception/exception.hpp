#ifndef __AHL_EXCEPTION_EXCEPTION_HPP
#define __AHL_EXCEPTION_EXCEPTION_HPP

#include <sstream>

namespace ahl
{

  class Exception
  {
  public:
    explicit Exception(const std::string& src, const std::string& msg)
      : src_(src), msg_(msg) throw() {}
    ~virtual Exception() {}

    virtual const char* what() const throw()
    {
      ss_ << "ahl::Exception occurred." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return ss_.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
    std::stringstream ss_;
  };

}

#endif /* __AHL_EXCEPTION_EXCEPTION_HPP */
