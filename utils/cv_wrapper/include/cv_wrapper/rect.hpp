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
