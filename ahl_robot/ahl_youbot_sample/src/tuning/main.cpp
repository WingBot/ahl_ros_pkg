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
  boost::timer t;
  bool applied = false;

  std::vector<youbot::JointTorqueSetpoint> tau_v(DOF);

  std::cout << "Start to apply joint effort ... ";
  t.restart();

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

    double time = t.elapsed();
    recordJointEffort(joint_idx, time, tau);

    if(time > sec)
    {
      break;
    }

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

  int joint_idx = 4;
  double current = 1.0;
  double sec = 3.0;

  Kp[0] = 1200;
  Kp[1] = 1200;
  Kp[2] = 1200;
  Kp[3] = 2000;
  Kp[4] = 4000;

  Ki[0] = 3000;
  Ki[1] = 3000;
  Ki[2] = 3000;
  Ki[3] = 4000;
  Ki[4] = 4000;

  Kd[0] = 0;
  Kd[1] = 0;
  Kd[2] = 0;
  Kd[3] = 0;
  Kd[4] = 0;

  setPIDGains();
  readPIDGains();

  std::cout << joint_idx << ", " << current << ", " << sec << std::endl;
  applyJointEffort(joint_idx, current, sec);

  applyJointEffort(joint_idx, 0.0, sec);

  return 0;
}
