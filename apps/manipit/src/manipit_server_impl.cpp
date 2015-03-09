#include "manipit/manipit_server_impl.hpp"

using namespace manipit;

ManipitServerImpl::ManipitServerImpl()
{
  manipit_ = ManipitPtr(new Manipit());
}

bool ManipitServerImpl::init(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& rse)
{
  ROS_INFO_STREAM("init");
  return true;
}

bool ManipitServerImpl::start(std_srvs::Empty::Request& req,
                              std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("start");
  return true;
}

bool ManipitServerImpl::stop(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM("stop");
  return true;
}
