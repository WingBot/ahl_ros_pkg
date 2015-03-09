#ifndef __MANIPIT_HAND_DETECTOR_HAND_DETECTOR_HPP
#define __MANIPIT_HAND_DETECTOR_HAND_DETECTOR_HPP

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace manipit
{

  class HandDetector
  {
  public:
    virtual ~HandDetector() {}

    virtual bool detect(const cv::Mat& rgb, const cv::Mat& depth, cv::Rect& hand_roi) = 0;

  private:

  };

  typedef boost::shared_ptr<HandDetector> HandDetectorPtr;
}

#endif /* __MANIPIT_HAND_DETECTOR_HAND_DETECTOR_HPP */
