#include "manipit/manipit_server.hpp"

using namespace manipit;

ManipitServer::ManipitServer()
{
  ros::NodeHandle nh;

  srv_init_ = nh.advertiseService(
    "manipit/init", &ManipitServer::init, this);
  srv_start_ = nh.advertiseService(
    "manipit/start", &ManipitServer::start, this);
  srv_stop_ = nh.advertiseService(
    "manipit/stop", &ManipitServer::stop, this);

  impl_ = ManipitServerImplPtr(new ManipitServerImpl());
}

bool ManipitServer::init(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& res)
{
  return impl_->init(req, res);
}

bool ManipitServer::start(std_srvs::Empty::Request& req,
                          std_srvs::Empty::Response& res)
{
  return impl_->start(req, res);
}

bool ManipitServer::stop(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& res)
{
  return impl_->stop(req, res);
}
