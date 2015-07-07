#include <vector>
#include <ros/ros.h>
#include <vrep_common/JointSetStateData.h>
#include <ahl_vrep_bridge/vrep_bridge.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ahl_vrep_interface_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<vrep_common::JointSetStateData>(
    "/vrep/set_joint_states", 10);

  ahl_vrep::VrepBridgePtr vrep_bridge;
  vrep_bridge = ahl_vrep::VrepBridgePtr(new ahl_vrep::VrepBridge());
  vrep_bridge->addJoint("youBotArmJoint0");
  vrep_bridge->addJoint("youBotArmJoint1");
  vrep_bridge->addJoint("youBotArmJoint2");
  vrep_bridge->addJoint("youBotArmJoint3");
  vrep_bridge->addJoint("youBotArmJoint4");
  vrep_bridge->call();

  ros::Rate r(1000.0);

  std::vector<int> handle;
  handle.push_back(vrep_bridge->getObjectHandle("youBotArmJoint0"));
  handle.push_back(vrep_bridge->getObjectHandle("youBotArmJoint1"));
  handle.push_back(vrep_bridge->getObjectHandle("youBotArmJoint2"));
  handle.push_back(vrep_bridge->getObjectHandle("youBotArmJoint3"));
  handle.push_back(vrep_bridge->getObjectHandle("youBotArmJoint4"));

  vrep_common::JointSetStateData msg;
  msg.handles.data.resize(handle.size());
  msg.setModes.data.resize(handle.size());
  msg.values.data.resize(handle.size());

  while(ros::ok())
  {
    for(unsigned int i = 0; i < handle.size(); ++i)
    {

    }

    r.sleep();
  }
}
