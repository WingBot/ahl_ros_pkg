#ifndef __MANIPIT_MANIPIT_SERVER_IMPL_HPP
#define __MANIPIT_MANIPIT_SERVER_IMPL_HPP

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "manipit/manipit.hpp"

namespace manipit
{

  class ManipitServerImpl
  {
  public:
    ManipitServerImpl();

    bool init(std_srvs::Empty::Request& req,
              std_srvs::Empty::Response& res);
    bool start(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& res);
    bool stop(std_srvs::Empty::Request& req,
              std_srvs::Empty::Response& res);
  private:
    ManipitPtr manipit_;
  };

  typedef boost::shared_ptr<ManipitServerImpl> ManipitServerImplPtr;
}

#endif /* __MANIPIT_MANIPIT_SERVER_IMPL_HPP */
