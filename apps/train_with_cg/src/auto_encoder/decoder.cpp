#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <neural_network/neural_network.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "decoder");
  ros::NodeHandle nh;

  std::string config_full_path;
  std::string data_in_path;
  std::string data_name;
  std::string extension;

  nh.param<std::string>("path/config", config_full_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/results/auto_encoder/auto_encoder0_result.yaml");
  nh.param<std::string>("path/data_in", data_in_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/data/depth");
  nh.param<std::string>("data_name", data_name, "image");
  nh.param<std::string>("extension", extension, "pgm");

  using namespace nn;

  ConfigPtr config = ConfigPtr(new Config());
  NeuralNetworkPtr nn = NeuralNetworkPtr(new NeuralNetwork());

  config->init(config_full_path);
  config->print();

  nn->init(config);

  unsigned long idx = 0;

  while(ros::ok())
  {
    std::stringstream name;
    name << data_in_path << "/" << data_name << idx << "." << extension;
    ++idx;

    std::cout << name.str() << std::endl;

    cv::Mat src = cv::imread(name.str(), CV_8UC1);
    if(src.rows == 0 || src.cols == 0)
      break;

    Eigen::MatrixXd input(src.rows * src.cols, 1);

    for(unsigned int y = 0; y < src.rows; ++y)
    {
      uchar* ptr = src.ptr<uchar>(y);

      for(unsigned int x = 0; x < src.cols; ++x)
      {
        input.coeffRef(x + y * src.cols, 0) = static_cast<double>(ptr[x] / 255.0);
      }
    }

    Eigen::MatrixXd output = nn->getOutput(input);
    output *= 255;

    cv::Mat dst(src.rows, src.cols, CV_8UC1);

    for(unsigned int y = 0; y < dst.rows; ++y)
    {
      uchar* ptr = dst.ptr<uchar>(y);

      for(unsigned int x = 0; x < dst.cols; ++x)
      {
        double val = output.coeff(x + y * dst.cols, 0);
        ptr[x] = static_cast<uchar>(val);
      }
    }

    cv::Mat resized_src(src.rows * 10, src.cols * 10, CV_8UC1);
    cv::resize(src, resized_src, resized_src.size());

    cv::imshow("src", resized_src);

    cv::Mat resized_dst(dst.rows * 10, dst.cols * 10, CV_8UC1);
    cv::resize(dst, resized_dst, resized_dst.size());

    cv::imshow("dst", resized_dst);

    if(cv::waitKey(100) == 27)
    {
      break;
    }
  }

  return 0;
}
