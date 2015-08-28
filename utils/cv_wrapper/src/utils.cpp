/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Daichi Yoshikawa
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Daichi Yoshikawa nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Daichi Yoshikawa
 *
 *********************************************************************/

#include "cv_wrapper/Labeling.h"
#include "cv_wrapper/utils.hpp"

namespace cv_wrapper
{

  bool isEmpty(const cv::Mat& img)
  {
    return (img.rows == 0) | (img.cols == 0);
  }

  bool imshow(const std::string& name, const cv::Mat& img)
  {
    if(!isEmpty(img))
      return false;

    cv::imshow(name, img);
    return true;
  }

  char waitKey(double interval)
  {
    char key = cv::waitKey(interval);

    switch(key)
    {
    case 'q':
    case 27:
      exit(0);
      break;
    default:
      break;
    };

    return key;
  }

  void closing(const cv::Mat& src, cv::Mat& dst, unsigned int erodes, unsigned int dilates)
  {
    cv::erode (src, dst, cv::Mat(), cv::Point(-1, -1), erodes);
    cv::dilate(dst, dst, cv::Mat(), cv::Point(-1, -1), dilates);
  }

  void convertRGBToHSV(const cv::Mat& src, std::vector<cv::Mat>& hsv)
  {
    if(src.rows == 0 || src.cols == 0)
      return;

    cv::Mat temp(src.rows, src.cols, CV_8UC3);

    cv::cvtColor(src, temp, CV_BGR2HSV);
    cv::split(temp, hsv);
  }

  void extractDesignatedArea(cv::Mat& gray, cv::Mat& dst, int min, int max)
  {
    cv::Mat temp1(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat temp2(gray.rows, gray.cols, CV_8UC1, cv::Scalar(0));

    cv::threshold(gray, temp1, min, 255, cv::THRESH_BINARY);
    cv::threshold(gray, temp2, max, 255, cv::THRESH_BINARY_INV);

    temp1.copyTo(dst, temp2);
  }

  void extractDesignatedHueArea(cv::Mat& hue, cv::Mat& dst, int min, int max)
  {
    if(min > 255 && max > 255)
    {
      min -= 255;
      max -= 255;
      extractDesignatedArea(hue, dst, min, max);
      return;
    }
    else if(min <= 255 && max <= 255)
    {
      extractDesignatedArea(hue, dst, min, max);
      return;
    }
    else if(min > 255 && max <= 255)
    {
      cv::Mat temp1(hue.rows, hue.cols, CV_8UC1, cv::Scalar(0));
      cv::Mat temp2(hue.rows, hue.cols, CV_8UC1, cv::Scalar(0));

      int min2 = min - 255;

      cv::threshold(hue, temp1, min2, 255, cv::THRESH_BINARY);
      cv::threshold(hue, temp2, max, 255, cv::THRESH_BINARY_INV);
      cv::bitwise_or(temp1, temp2, dst);
    }
    else //min <= 255 && max > 255                                                                                        
    {
      cv::Mat temp1(hue.rows, hue.cols, CV_8UC1, cv::Scalar(0));
      cv::Mat temp2(hue.rows, hue.cols, CV_8UC1, cv::Scalar(0));

      int max2 = max - 255;

      cv::threshold(hue, temp1, min, 255, cv::THRESH_BINARY);
      cv::threshold(hue, temp2, max2, 255, cv::THRESH_BINARY_INV);
      cv::bitwise_or(temp1, temp2, dst);
    }
  }

  void extractDesignatedSatArea(cv::Mat& sat, cv::Mat& dst, int min, int max)
  {
    extractDesignatedArea(sat, dst, min, max);
  }

  void extractDesignatedValArea(cv::Mat& val, cv::Mat& dst, int min, int max)
  {
    extractDesignatedArea(val, dst, min, max);
  }

  void extractDesignatedHSVArea(std::vector<cv::Mat>& hsv, cv::Mat& dst,
                                int h_min, int h_max,
                                int s_min, int s_max,
                                int v_min, int v_max,
                                bool labeling, int erode_num, int dilate_num)
  {
    cv::Mat h_mask(hsv[0].rows, hsv[0].cols, CV_8UC1, cv::Scalar(0));
    cv::Mat s_mask(hsv[1].rows, hsv[1].cols, CV_8UC1, cv::Scalar(0));
    cv::Mat v_mask(hsv[2].rows, hsv[2].cols, CV_8UC1, cv::Scalar(0));

    cv_wrapper::extractDesignatedHueArea(hsv[0], h_mask, h_min, h_max);
    cv_wrapper::closing(h_mask, h_mask, erode_num, dilate_num);
    cv_wrapper::extractDesignatedSatArea(hsv[1], s_mask, s_min, s_max);
    cv_wrapper::closing(s_mask, s_mask, erode_num, dilate_num);
    cv_wrapper::extractDesignatedValArea(hsv[2], v_mask, v_min, v_max);
    cv_wrapper::closing(v_mask, v_mask, erode_num, dilate_num);

    cv::bitwise_and(h_mask, s_mask, dst);
    cv::bitwise_and(v_mask, dst, dst);

    if(labeling)
    {
      cv_wrapper::labeling(dst, dst);
    }
  }

  void labeling(cv::Mat& src, cv::Mat& dst)
  {
    if(src.rows % 2 != 0 || src.cols % 2 != 0)
    {
      throw("cv_wrapper::labeling", "src width or height is not an even number.");
    }

    cv::Mat label(src.size(), CV_16SC1);

    LabelingBS labeling;
    labeling.Exec(src.data, (short*)label.data, src.cols, src.rows, true, 0);

    cv::Mat labelarea(src.size(), CV_8UC1, cv::Scalar(0));
    cv::compare(label, 1, labelarea, CV_CMP_EQ);

    if(src.type() == CV_8UC1)
    {
      labelarea.copyTo(dst);
    }
    else if(src.type() == CV_8UC3)
    {
      cv::Mat color(src.size(), CV_8UC3, cv::Scalar(255, 255, 255));
      color.copyTo(dst, labelarea);
    }
    else
    {
      throw("cv_wrapper::labeling", "Not supported src.type() was used.");
    }
  }

}
