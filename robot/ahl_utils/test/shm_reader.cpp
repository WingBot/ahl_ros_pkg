#include <ros/ros.h>
#include "ahl_utils/shared_memory.hpp"

using namespace ahl_utils;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "shm_reader");
  ros::NodeHandle nh;

  SharedMemory<double>::Ptr shm = SharedMemory<double>::Ptr(new SharedMemory<double>("shm"));

  ros::Rate r(1000.0);

  double data = 0;
  while(ros::ok())
  {
    shm->read(data);
    std::cout << data << std::endl;
    r.sleep();
  }

  return 0;
}
