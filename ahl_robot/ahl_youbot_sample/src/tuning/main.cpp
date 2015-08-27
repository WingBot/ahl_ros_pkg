#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include <youbot_driver/youbot/YouBotManipulator.hpp>
#include <ros/ros.h>

typedef boost::shared_ptr<youbot::YouBotManipulator> YouBotManipulatorPtr;
YouBotManipulatorPtr Manipulator;

const unsigned int DOF = 5;
int Kp[5];
int Ki[5];
int Kd[5];
youbot::PParameterCurrentControl PParam;
youbot::IParameterCurrentControl IParam;
youbot::DParameterCurrentControl DParam;

void readPIDGains()
{
  std::cout << "Read param." << std::endl;

  for(int i = 0; i < DOF; ++i)
  {
    int tmp;

    Manipulator->getArmJoint(i + 1).getConfigurationParameter(PParam);
    Manipulator->getArmJoint(i + 1).getConfigurationParameter(IParam);
    Manipulator->getArmJoint(i + 1).getConfigurationParameter(DParam);

    PParam.getParameter(tmp);
    Kp[i] = tmp;

    IParam.getParameter(tmp);
    Ki[i] = tmp;

    DParam.getParameter(tmp);
    Kd[i] = tmp;
  }

  for(unsigned int i = 0; i < DOF; ++i)
  {
    std::cout << "Kp[" << i + 1 << "] = " << Kp[i] << std::endl;
    std::cout << "Ki[" << i + 1 << "] = " << Ki[i] << std::endl;
    std::cout << "Kd[" << i + 1 << "] = " << Kd[i] << std::endl;
  }
}

void setPIDGains()
{
  for(unsigned int i = 0; i < DOF; ++i)
  {
    PParam.setParameter(Kp[i]);
    Manipulator->getArmJoint(i + 1).setConfigurationParameter(PParam);

    IParam.setParameter(Ki[i]);
    Manipulator->getArmJoint(i + 1).setConfigurationParameter(IParam);

    DParam.setParameter(Kd[i]);
    Manipulator->getArmJoint(i + 1).setConfigurationParameter(DParam);
  }
}

void recordJointEffort(int idx, double t, double tau_ref)
{
  static std::ofstream ofs("joint_effort.data");
  std::vector<youbot::JointSensedTorque> tau;
  Manipulator->getJointData(tau);

  ofs << t << " " << tau_ref << " " << tau[idx].torque.value() << std::endl;
}

void applyJointEffort(int joint_idx, double tau, double sec)
{
  //boost::timer t;
  bool applied = false;

  std::vector<youbot::JointTorqueSetpoint> tau_v(DOF);

  std::cout << "Start to apply joint effort ... ";
  //t.restart();

  ros::Rate r(100);

  double start = ros::Time::now().toNSec() * 0.001 * 0.001 * 0.001;

  while(true)
  {
    if(!applied)
    {
      //apply joint effort
      for(unsigned int i = 0; i < DOF; ++i)
      {
        tau_v[i].torque = 0.0 * newton_meter;
      }

      tau_v[joint_idx].torque = tau * newton_meter;
      Manipulator->setJointData(tau_v);
      applied = true;
    }

    //double time = t.elapsed();
    double time = ros::Time::now().toNSec() * 0.001 * 0.001 * 0.001 - start;
    recordJointEffort(joint_idx, time, tau);

    if(time > sec)
    {
      break;
    }

    std::cout << time << std::endl;
    ros::Duration(0.001).sleep();
  }

  std::cout << "Finish" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tuning");
  ros::NodeHandle nh;

  Manipulator = YouBotManipulatorPtr(
    new youbot::YouBotManipulator("youbot-manipulator", YOUBOT_CONFIGURATIONS_DIR));

  Manipulator->doJointCommutation();
  Manipulator->calibrateManipulator(true);

  readPIDGains();

  int joint_idx = 2;
  double current = -5.0;
  double sec = 1.0;

  Kp[0] = 1800; // 2200 was the best -> 1800
  Kp[1] = 1600; // 1600 was the best -> 1600
  Kp[2] = 1600; // 1600 was the best -> 1600
  Kp[3] = 1900; // 1900 was the best -> 1900
  Kp[4] = 5000; // 10000 was the best -> 5000

  Ki[0] = 6000; // 6000 was the best
  Ki[1] = 5000; // 5000 was the best
  Ki[2] = 5500; // 5500 was the best
  Ki[3] = 6000; // 6000 was the best
  Ki[4] = 8000; // 8000 was the best

  Kd[0] = 0;
  Kd[1] = 0;
  Kd[2] = 0;
  Kd[3] = 0;
  Kd[4] = 0;

  setPIDGains();
  readPIDGains();

  std::cout << joint_idx << ", " << current << ", " << sec << std::endl;
  applyJointEffort(joint_idx, 0.0, 1.0);
  //applyJointEffort(joint_idx, 0.2, sec);
  //applyJointEffort(joint_idx, 0.0, 1.0);
  applyJointEffort(joint_idx, 5.0, 10.0);
  applyJointEffort(joint_idx, 0.0, 1.0);
  applyJointEffort(joint_idx, -5.0, 10.0);
  applyJointEffort(joint_idx, 0.0, 10.0);

  return 0;
}
