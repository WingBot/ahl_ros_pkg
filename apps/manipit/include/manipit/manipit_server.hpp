#ifndef __MANIPIT_MANIPIT_SERVER_HPP
#define __MANIPIT_MANIPIT_SERVER_HPP

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "manipit/manipit_server_impl.hpp"

namespace manipit
{

  class ManipitServer
  {
  public:
    ManipitServer();

  private:
    bool init(std_srvs::Empty::Request& req,
              std_srvs::Empty::Response& res);
    bool start(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& res);
    bool stop(std_srvs::Empty::Request& req,
              std_srvs::Empty::Response& res);

    boost::mutex mutex_;

    ros::ServiceServer srv_init_;
    ros::ServiceServer srv_start_;
    ros::ServiceServer srv_stop_;

    ManipitServerImplPtr impl_;
  };

  typedef boost::shared_ptr<ManipitServer> ManipitServerPtr;
}

#endif /* __MANIPIT_MANIPIT_SERVER_HPP */
