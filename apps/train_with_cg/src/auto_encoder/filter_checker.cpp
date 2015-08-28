#include <string>
#include <ctime>
#include <sstream>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ahl_utils/yaml_loader.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_checker");
  ros::NodeHandle nh("~");

  std::string yaml_name;
  nh.param<std::string>("yaml_name", yaml_name,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/apps/train_with_cg/results/auto_encoder/auto_encoder0_result.yaml");

  int h, w;
  nh.param<int>("image/height", h, 32);
  nh.param<int>("image/width",  w, 32);

  ahl_utils::YAMLLoader loader(yaml_name);
  Eigen::MatrixXd tmp;
  loader.loadMatrix("weight0", tmp);

  Eigen::MatrixXd weight = tmp.transpose();
  std::cout << "weight size : " << weight.rows() << ", " << weight.cols() << std::endl;

  for(unsigned int i = 0; i < weight.cols(); ++i)
  {
    Eigen::MatrixXd img_mat = weight.block(0, i, weight.rows() - 1, weight.cols());
    cv::Mat img(h, w, CV_8UC1);

    double max = img_mat.maxCoeff();
    double min = img_mat.minCoeff();

    for(unsigned int y = 0; y < h; ++y)
    {
      uchar* ptr = img.ptr<uchar>(y);

      for(unsigned int x = 0; x < w; ++x)
      {
        double val = img_mat.coeff(x + y * w, 0);
        val -= min;
        val /= (max - min);
        val *= 255.0;

        ptr[x] = val;
      }
    }

    if(img.rows == 0 || img.cols == 0)
      break;

    std::stringstream img_name;
    std::stringstream resized_img_name;

    img_name << "filter" << i;
    resized_img_name << "resized_filter" << i;

    cv::Mat resized_img(img.rows * 10, img.cols * 10, CV_8UC1);
    cv::resize(img, resized_img, resized_img.size());

    cv::imshow("filter", img);
    cv::imshow("resized_filter", resized_img);

    if(cv::waitKey(100) == 27)
    {
      break;
    }
  }

  return 0;
}
