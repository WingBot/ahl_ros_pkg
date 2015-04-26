#ifndef __TRAIN_WITH_CG_EXCEPTIONS_HPP
#define __TRAIN_WITH_CG_EXCEPTIONS_HPP

#include <sstream>

namespace train
{

  class Exception
  {
  public:
    explicit Exception(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg) {}

    const char* what() const throw()
    {
      std::stringstream msg;
      msg << "train::Exception was thrown." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return msg.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

  class FatalException
  {
  public:
    explicit FatalException(const std::string& src, const std::string& msg) throw()
      : src_(src), msg_(msg) {}

    const char* what() const throw()
    {
      std::stringstream msg;
      msg << "train::Exception was thrown." << std::endl
          << "  src : " << src_ << std::endl
          << "  msg : " << msg_;

      return msg.str().c_str();
    }

  private:
    std::string src_;
    std::string msg_;
  };

}

#endif /* __TRAIN_WITH_CG_EXCEPTIONS_HPP */
