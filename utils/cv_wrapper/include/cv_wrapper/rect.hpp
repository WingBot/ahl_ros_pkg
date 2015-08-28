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

#ifndef __CV_WRAPPER_RECT_HPP
#define __CV_WRAPPER_RECT_HPP

#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

namespace cv_wrapper
{

  class Rect
  {
  public:
    Rect() {}
    Rect(int image_w, int image_h)
      : image_w_(image_w), image_h_(image_h) {}
    Rect(int image_w, int image_h, int x, int y, int w, int h)
      : image_w_(image_w), image_h_(image_h_), rect_(x, y, w, h) {}

    void draw(cv::Mat& image, int r = 0, int g = 0, int b = 255);
    void set(int x, int y, int w, int h);
    void bound(cv::Mat& src, unsigned int pad, bool white_is_ignored = true, bool square = false);
    void printInfo();

    double getAspectRatio()
    {
      return 1.0 * rect_.width / rect_.height;
    }

    const int getX() const
    {
      return rect_.x;
    }
    const int getY() const
    {
      return rect_.y;
    }
    const int getW() const
    {
      return rect_.width;
    }
    const int getH() const
    {
      return rect_.height;
    }

    const cv::Rect& getRect() const
    {
      return rect_;
    }

    bool isZero();

  private:
    void limit(const cv::Mat& img);
    bool pixelIsNot(uchar* psrc, int channels, int ignored_value);
    int getUpperY(cv::Mat& src, uchar ignored_value);
    int getLowerY(cv::Mat& src, uchar ignored_value);
    int getLeftX (cv::Mat& src, uchar ignored_value);
    int getRightX(cv::Mat& src, uchar ignored_value);

    void convertSquare();

    int image_w_;
    int image_h_;
    cv::Rect rect_;
  };
}

#endif /* __CV_WRAPPER_RECT_HPP */
