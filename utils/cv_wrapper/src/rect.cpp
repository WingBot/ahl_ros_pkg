#include <ros/ros.h>
#include "cv_wrapper/rect.hpp"

using namespace cv_wrapper;

void Rect::draw(cv::Mat& image, int r, int g, int b)
{
  cv::rectangle(image, cv::Point(rect_.x, rect_.y), cv::Point(rect_.x + rect_.width, rect_.y + rect_.height), cv::Scalar(r, g, b));
}

void Rect::set(int x, int y, int w, int h)
{
  rect_.x = x;
  rect_.y = y;
  rect_.width = w;
  rect_.height = h;

  //limit();
}

void Rect::limit(const cv::Mat& img)
{
  //check origin
  if(rect_.x < 0)
  {
    rect_.x = 0;
  }
  if(rect_.x >= img.cols)
  {
    rect_.x = img.cols - 1;
  }

  if(rect_.y < 0)
  {
    rect_.y = 0;
  }
  if(rect_.y >= img.rows)
  {
    rect_.y = img.rows - 1;
  }

  //check length
  if(rect_.width < 0)
  {
    rect_.width = 0;
  }
  if(rect_.height < 0)
  {
    rect_.height = 0;
  }
  int end_x = rect_.x + rect_.width;
  int end_y = rect_.y + rect_.height;

  if(end_x > img.cols)
  {
    rect_.width -= (end_x - img.cols) + 1;
  }
  if(end_y > img.rows)
  {
    rect_.height -= (end_y - img.rows) + 1;
  }
}

//TODO : a bound function & related functions may include some bugs...
void Rect::bound(cv::Mat& src, unsigned int pad, bool white_is_ignored, bool square)
{
  int height = src.rows;
  int width  = src.cols;

  int upper_y = 0;
  int lower_y = height;
  int left_x  = 0;
  int right_x = width;

  uchar ignored_value = 0;
  if(white_is_ignored)
  {
    ignored_value = 255;
  }

  upper_y = getUpperY(src, ignored_value);
  lower_y = getLowerY(src, ignored_value);
  left_x  = getLeftX (src, ignored_value);
  right_x = getRightX(src, ignored_value);

  rect_.x = left_x - pad;
  rect_.y = upper_y - pad;
  rect_.width  = right_x - left_x + 2 * pad;
  rect_.height = lower_y - upper_y + 2 * pad;

  if(square)
  {
    convertSquare();
  }

  limit(src);
}

void Rect::printInfo()
{
  std::cout << "(x, y) = ("
            << rect_.x
            << " ,"
            << rect_.y
            << "), (h, w) = ("
            << rect_.height
            << ", "
            << rect_.width
            << ")"
            << std::endl;
}

bool Rect::isZero()
{
  if(rect_.width == 0 || rect_.height == 0)
    return true;
  return false;
}

bool Rect::pixelIsNot(uchar* psrc, int channels, int ignored_value)
{
  if(channels == 1)
  {
    if(*psrc != ignored_value)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if(channels == 3)
  {
    if(psrc[0] != ignored_value
       || psrc[1] != ignored_value
       || psrc[2] != ignored_value)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else if(channels == 4)
  {
    if(psrc[0] != ignored_value
       || psrc[1] != ignored_value
       || psrc[2] != ignored_value
       || psrc[3] != ignored_value)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

int Rect::getUpperY(cv::Mat& src, uchar ignored_value)
{
  int channels = src.channels();
  int height   = src.rows;
  int width    = src.cols*channels;

  for(int y = 0; y < height; ++y)
  {
    uchar* psrc = src.ptr<uchar>(y);

    for(int x = 0; x < width; x += channels)
    {
      if(pixelIsNot(&psrc[x], channels, ignored_value))
      {
        if(y > 0)
        {
          return y - 1;
        }
        else
        {
          return y;
        }
      }
    }
  }

  return 0;
}

int Rect::getLowerY(cv::Mat& src, uchar ignored_value)
{
  int channels = src.channels();
  int height   = src.rows;
  int width    = src.cols*channels;

  for(int y = height - 1; y >= 0; --y)
  {
    uchar* psrc = src.ptr<uchar>(y);

    for(int x = 0; x < width; x += channels)
    {
      if(pixelIsNot(&psrc[x], channels, ignored_value))
      {
        if(y < height - 1)
        {
          return y + 1;
        }
        else
        {
          return y;
        }
      }
    }
  }

  return height - 1;
}

int Rect::getLeftX (cv::Mat& src, uchar ignored_value)
{
  int channels = src.channels();
  int height   = src.rows;
  int width    = src.cols*channels;

  for(int x = 0; x < width; x += channels)
  {
    for(int y = 0; y < height; ++y)
    {
      uchar* psrc = src.ptr<uchar>(y);
      if(pixelIsNot(&psrc[x], channels, ignored_value))
      {
        if(x > 0)
        {
          return x/channels - 1;
        }
        else
        {
          return x/channels;
        }
      }
    }
  }

  return 0;
}

int Rect::getRightX(cv::Mat& src, uchar ignored_value)
{
  int channels = src.channels();
  int height   = src.rows;
  int width    = src.cols*channels;

  for(int x = width - channels; x >= 0; x -= channels)
  {
    for(int y = 0; y < height; ++y)
    {
      uchar* psrc = src.ptr<uchar>(y);
      if(pixelIsNot(&psrc[x], channels, ignored_value))
      {
        if(x < width - 1)
        {
          return x/channels + 1;
        }
        else
        {
          return x/channels;
        }
      }
    }
  }

  return width - 1;
}

//TODO : fix bug
void Rect::convertSquare()
{
  int def = rect_.height - rect_.width;
  bool vertically_long = def > 0 ? true : false;

  if(vertically_long)
  {
    int x_def = def / 2;

    rect_.x -= x_def;
    rect_.width += def;
  }
  else
  {
    int y_def = std::abs(def/2);

    rect_.y -= y_def;
    rect_.height += std::abs(def);
  }
}
