#include <Eigen/Dense>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel");
  ros::NodeHandle nh;

  const unsigned int procs = omp_get_num_procs();
  const unsigned int sep_size = 100;
  const unsigned int size = procs * sep_size;

  std::vector<Eigen::VectorXd> data(size);
  std::vector<Eigen::VectorXd> result(size);

  for(unsigned int i = 0; i < data.size(); ++i)
  {
    data[i]   = Eigen::VectorXd::Constant(i, size);
    result[i] = Eigen::VectorXd::Constant(0.1, size);
  }

  std::vector< std::vector<Eigen::VectorXd> > sep_data(procs);
  std::vector< std::vector<Eigen::VectorXd> > sep_result(procs);

  for(unsigned int i = 0; i < procs; ++i)
  {
    for(unsigned int j = 0; j < sep_size; ++j)
    {
      sep_data[i].push_back(data[i * sep_size + j]);
    }
  }

  omp_set_num_threads(procs);

  Eigen::MatrixXd m1(100, 100);
  Eigen::MatrixXd m2(100, 100);
  Eigen::MatrixXd m3(100, 100);
  Eigen::MatrixXd m4(100, 100);

  #pragma omp parallel num_threads(3)
  #pragma omp sections
  {

    #pragma omp section
    //for(unsigned int i = 0; i < 100; ++i)
    {
      static unsigned long cnt = 0;

      while(ros::ok())
      {
        m1 = Eigen::MatrixXd::Random(100, 100) * Eigen::MatrixXd::Random(100, 100);
        ++cnt;
        std::cout << "1, " << cnt << std::endl;
      }
    }

    #pragma omp section
    //for(unsigned int i = 0; i < 100; ++i)
    {
      static unsigned long cnt = 0;

      while(ros::ok())
      {
        m2 = Eigen::MatrixXd::Random(100, 100) * Eigen::MatrixXd::Random(100, 100);
        ++cnt;
        std::cout << "2, " << cnt << std::endl;
      }
    }

    #pragma omp section
    //for(unsigned int i = 0; i < 100; ++i)
    {
      static unsigned long cnt = 0;

      while(ros::ok())
      {
        m3 = Eigen::MatrixXd::Random(100, 100) * Eigen::MatrixXd::Random(100, 100);
        ++cnt;
        std::cout << "3, " << cnt << std::endl;
      }
    }

  }
  #pragma omp end parallel

  return 0;
}
