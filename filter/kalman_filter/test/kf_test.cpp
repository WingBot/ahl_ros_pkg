#include <iostream>
#include <fstream>
#include <ctime>

#include <ros/ros.h>

#include "kalman_filter/kalman_filter.hpp"
#include "kalman_filter/normal_distribution.hpp"
#include "kalman_filter/exception.hpp"

static const double SAMPLING_TIME = 0.10;
static const double FREQUENCY     = 0.05;
static const double ITERATIONS    = 200;

void initMomentParameters(Eigen::MatrixXd& init_mean, Eigen::MatrixXd& init_variance,
                          Eigen::MatrixXd& uncertainty_mean, Eigen::MatrixXd& uncertainty_variance,
                          Eigen::MatrixXd& msr_noise_mean, Eigen::MatrixXd& msr_noise_variance)
{
  init_mean.resize(2, 1);
  init_variance.resize(2, 2);

  init_mean << 1.0, 0.0;
  init_variance << 0.10, 0.05,
                   0.05, 0.10;

  uncertainty_mean.resize(2, 1);
  uncertainty_variance.resize(2, 2);

  uncertainty_mean << 0.0, 0.0;
  uncertainty_variance << 0.10, 0.05,
                          0.05, 0.10;

  msr_noise_mean.resize(2, 1);
  msr_noise_variance.resize(2, 2);

  msr_noise_mean << 1.0, 0.0;
  msr_noise_variance << 0.02, 0.005,
                        0.005, 0.02;
}

void initLinearModel(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C)
{
  A.resize(2, 2);
  A << 1.0, 0.0,
       0.0, 1.0;

  B.resize(2, 2);
  B << SAMPLING_TIME, 0.0,
       0.0, SAMPLING_TIME;

  C.resize(2, 2);
  C << 1.0, 0.0,
       0.0, 1.0;
}

void updateCtrlDataAndMsrData(unsigned int idx, Eigen::MatrixXd& u, Eigen::MatrixXd& z)
{
  double t = SAMPLING_TIME * idx;
  double angle = 2.0 * M_PI * FREQUENCY * t;

  u.resize(2, 1);
  u << -sin(angle), cos(angle);

  z.resize(2, 1);

  const double max = 0.2;
  double rand1 = std::rand() % 1000 / 1000.0 * 0.04;
  double rand2 = std::rand() % 1000 / 1000.0 * 0.04;
  z << cos(angle) + rand1, sin(angle) + rand2;
}

int main(int argc, char** argv)
{
  std::srand(static_cast<unsigned int>(std::time(NULL)));

  try
  {
    ros::init(argc, argv, "kf_test");
    ros::NodeHandle nh;

    std::ofstream ofs1("predicted_state.txt");
    if(ofs1.fail())
    {
      ROS_ERROR_STREAM("Could not open file.");
      return -1;
    }

    std::ofstream ofs2("state.txt");
    if(ofs2.fail())
    {
      ROS_ERROR_STREAM("Could not open file.");
      return -1;
    }

    using namespace kf;
    KalmanFilterPtr kalman_filter = KalmanFilterPtr(new KalmanFilter());

    Eigen::MatrixXd init_mean;
    Eigen::MatrixXd init_variance;

    Eigen::MatrixXd uncertainty_mean;
    Eigen::MatrixXd uncertainty_variance;

    Eigen::MatrixXd msr_noise_mean;
    Eigen::MatrixXd msr_noise_variance;

    ::initMomentParameters(init_mean, init_variance,
                           uncertainty_mean, uncertainty_variance,
                           msr_noise_mean, msr_noise_variance);

    NormalDistributionPtr init_state
      = NormalDistributionPtr(new NormalDistribution(init_mean, init_variance));
    NormalDistributionPtr uncertainty
      = NormalDistributionPtr(new NormalDistribution(uncertainty_mean, uncertainty_variance));
    NormalDistributionPtr msr_noise
      = NormalDistributionPtr(new NormalDistribution(msr_noise_mean, msr_noise_variance));

    kalman_filter->setRandomVariables(init_state, uncertainty, msr_noise);

    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd C;

    ::initLinearModel(A, B, C);

    kalman_filter->setLinearModel(A, B, C);

    for(unsigned int i = 0; i < ITERATIONS; ++i)
    {
      Eigen::MatrixXd u;
      Eigen::MatrixXd z;
      NormalDistributionPtr state = NormalDistributionPtr(new NormalDistribution);

      ::updateCtrlDataAndMsrData(i, u, z);
      kalman_filter->estimate(u, z, state);

      for(unsigned int i = 0; i < kalman_filter->getPredictedState()->getMean().rows(); ++i)
      {
        ofs1 << kalman_filter->getPredictedState()->getMean().coeff(i) << ", ";
      }
      ofs1 << std::endl;

      for(unsigned int i = 0; i < kalman_filter->getState()->getMean().rows(); ++i)
      {
        ofs2 << kalman_filter->getState()->getMean().coeff(i) << ", ";
      }
      ofs2 << std::endl;
    }
  }
  catch(kf::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
  }

  return 0;
}
