#include <ros/ros.h>
#include "std_utils/io_utils.hpp"
#include "neural_network/neural_network.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nn_test");
  ros::NodeHandle nh;

  std::string config_path;
  std::string data_in_path;
  std::string data_out_path;
  std::string max_min_path;
  std::string result_path;

  nh.param<std::string>("/neural_network/test/path/config", config_path,
//    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/yaml/sample.yaml");
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_result.yaml");
  nh.param<std::string>("/neural_network/test/path/data_in", data_in_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_in.dat");
  nh.param<std::string>("/neural_network/test/path/data_out", data_out_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_out.dat");
  nh.param<std::string>("/neural_network/test/path/max_min", max_min_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_max_min.yaml");
  nh.param<std::string>("/neural_network/test/path/result", result_path,
    "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_result.yaml");

  try
  {
    nn::ConfigPtr config = nn::ConfigPtr(new nn::Config());
    nn::TrainingDataPtr data = nn::TrainingDataPtr(new nn::TrainingData());
    nn::ScalerPtr scaler = nn::ScalerPtr(new nn::Scaler());
    nn::NeuralNetworkPtr nn = nn::NeuralNetworkPtr(new nn::NeuralNetwork());

    config->init(config_path);
    data->init(data_in_path, data_out_path);

    scaler->init(data);
    config->print();
    scaler->save(max_min_path);

    scaler->normalize(data);

    Eigen::MatrixXd input;
    input.resize(2, 1);
    input.coeffRef(0, 0) = 8.0;
    input.coeffRef(1, 0) = 0.0;

    scaler->normalizeInput(input);
    std_utils::IOUtils::print(input);

    std::vector<nn::TrainingDataPtr> data_array;

    nn->init(config);

    if(config->enableBackPropagation())
    {
      nn->train(data, result_path);
    }

    Eigen::MatrixXd test_output = nn->getOutput(input);
    scaler->denormalizeOutput(test_output);
    std_utils::IOUtils::print(test_output);
  }
  catch(nn::Exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    exit(1);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("Unknown exception was thrown.");
    exit(1);
  }

  return 0;
}
