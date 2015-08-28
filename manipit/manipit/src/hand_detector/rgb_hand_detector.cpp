#include <cv_wrapper/utils.hpp>
#include <cv_wrapper/hsv_mask.hpp>
#include <cv_wrapper/hsv_mask_param.hpp>
#include "manipit/hand_detector/rgb_hand_detector.hpp"

using namespace manipit;

bool RGBHandDetector::detect(const cv::Mat& rgb, const cv::Mat& depth, cv::Rect& hand_roi)
{
  using namespace cv_wrapper;
  HSVMask mask(rgb.rows, rgb.cols);
  HSVMaskParamPtr skin = HSVMaskParamPtr(new HSVMaskParam(0, 30, 50, 255, 50, 255));
  mask.add(skin);

  std::vector<cv::Mat> hsv;
  convertRGBToHSV(rgb, hsv);

  mask.getMask(hsv, true, 2, 2);

  cv::imshow("mask", mask.get());

  return true;
}
