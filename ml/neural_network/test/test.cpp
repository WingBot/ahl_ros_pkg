#include <ros/ros.h>
#include "std_utils/io_utils.hpp"
#include "neural_network/neural_network.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "nn_test");
  ros::NodeHandle nh;

  try
  {
    nn::ConfigPtr config = nn::ConfigPtr(new nn::Config());
    nn::TrainingDataPtr data = nn::TrainingDataPtr(new nn::TrainingData());
    nn::ScalerPtr scaler = nn::ScalerPtr(new nn::Scaler());
    nn::NeuralNetworkPtr nn = nn::NeuralNetworkPtr(new nn::NeuralNetwork());

    config->init("/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/yaml/sample.yaml");
    data->init("/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_in.dat",
               "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_out.dat");

    scaler->init(data);
    config->print();
    scaler->save("/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_max_min.yaml");

    scaler->normalize(data);

    Eigen::MatrixXd input;
    input.resize(2, 1);
    input.coeffRef(0, 0) = 8.0;
    input.coeffRef(1, 0) = 0.0;

    scaler->normalizeInput(input);
    std_utils::IOUtils::print(input);

    std::vector<nn::TrainingDataPtr> data_array;

    nn->init(config);
    nn->train(data, "/home/daichi/Work/catkin_ws/src/ahl_ros_pkg/ml/neural_network/test/test_result.yaml");

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
