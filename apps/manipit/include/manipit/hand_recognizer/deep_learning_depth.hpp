#ifndef __MANIPIT_HAND_RECOGNIZER_DEEP_LEARNING_DEPTH_HPP
#define __MANIPIT_HAND_RECOGNIZER_DEEP_LEARNING_DEPTH_HPP

#include <neural_network/neural_network.hpp>
#include "manipit/hand_recognizer/hand_recognizer.hpp"

namespace manipit
{

  class DeepLearningDepth : public HandRecognizer
  {
  public:
    DeepLearningDepth();
  private:
    bool recognizePose(const cv::Mat& rgb, const cv::Mat& depth, geometry_msgs::PoseStamped& pose);

    nn::ConfigPtr config_;
    nn::NeuralNetworkPtr nn_;
  };

}

#endif /* __MANIPIT_HAND_RECOGNIZER_DEEP_LEARNING_DEPTH_HPP */



