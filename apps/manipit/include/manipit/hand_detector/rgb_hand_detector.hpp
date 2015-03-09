#ifndef __MANIPIT_HAND_DETECTOR_RGB_HAND_DETECTOR_HPP
#define __MANIPIT_HAND_DETECTOR_RGB_HAND_DETECTOR_HPP

#include "manipit/hand_detector/hand_detector.hpp"

namespace manipit
{

  class RGBHandDetector : public HandDetector
  {
  public:

    virtual bool detect(const cv::Mat& rgb, const cv::Mat& depth, cv::Rect& hand_roi);

  private:

  };

}

#endif /* __MANIPIT_HAND_DETECTOR_RGB_HAND_DETECTOR_HPP */
