#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <gl_wrapper/gl_wrapper.hpp>
#include <cv_wrapper/rect.hpp>
#include "train_with_cg/collect_image/depth_image_saver.hpp"

using namespace train;

DepthImageSaver::DepthImageSaver()
{
  ros::NodeHandle nh;

  nh.param<double>("hand/size", hand_size_, 0.18);
  nh.param<double>("hand/img/size", hand_img_size_, 40);

  near_ = gl_wrapper::Render::PARAM->z_near;
  far_  = gl_wrapper::Render::PARAM->z_far;
}

void DepthImageSaver::save(const std::string& name)
{
  int w = glutGet(GLUT_WINDOW_WIDTH);
  int h = glutGet(GLUT_WINDOW_HEIGHT);

  float* buf = new float[w * h];

  glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, buf);
  cv::Mat depth(h, w, CV_32FC1, cv::Scalar(0.0));

  float min = far_;

  for(unsigned int y = 0; y < h; ++y)
  {
    float* ptr = depth.ptr<float>(y);

    for(unsigned int x = 0; x < w; ++x)
    {
      float z = buf[x + y * w];
      ptr[x] = -(far_ * near_) / (z * (far_ - near_) - far_);

      if(near_ < ptr[x] && ptr[x] < far_)
      {
        if(ptr[x] < min)
          min = ptr[x];
      }
    }
  }

  cv::Mat img(h, w, CV_8UC1, cv::Scalar(0));

  for(unsigned int y = 0; y < h; ++y)
  {
    float* depth_ptr = depth.ptr<float>(y);
    uchar* img_ptr   = img.ptr<uchar>(y);

    for(unsigned int x = 0; x < w; ++x)
    {
      if(near_ < depth_ptr[x] && depth_ptr[x] < far_)
      {
        depth_ptr[x] -= min;
        img_ptr[x] = static_cast<uchar>(depth_ptr[x] / hand_size_ * 255);
      }
      else
      {
        img_ptr[x] = 0;
      }
    }
  }

  cv_wrapper::Rect rect;
  rect.bound(img, 2, false, true);
  std::cout << rect.getH() << ", " << rect.getW() << std::endl;

  cv::Mat roi_img = img(rect.getRect());
  cv::Mat resized_img(hand_img_size_, hand_img_size_, CV_8UC1);

  cv::resize(roi_img, resized_img, resized_img.size());

  cv::Mat dst;
  cv::flip(resized_img, dst, 0);

  cv::imwrite(name, dst);

  delete buf;
}
