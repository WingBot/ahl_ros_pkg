#ifndef __MANIPIT_HAND_RECOGNIZER_HAND_RECOGNIZER_HPP
#define __MANIPIT_HAND_RECOGNIZER_HAND_RECOGNIZER_HPP

#include <vector>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace manipit
{

  class HandRecognizer
  {
  public:
    enum MethodList
    {
      DeepLearningDepth,
      MethodNum,
    };

    virtual ~HandRecognizer() {}

    virtual bool recognizePose(const cv::Mat& rgb, const cv::Mat& depth, geometry_msgs::PoseStamped& pose) { return false; }
  };

  typedef boost::shared_ptr<HandRecognizer> HandRecognizerPtr;
}

#endif /* __MANIPIT_HAND_RECOGNIZER_HAND_RECOGNIZER_HPP */
